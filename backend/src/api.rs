use axum::{
    extract::{
        Path, State, WebSocketUpgrade, ws::{Message, WebSocket},
    },
    response::IntoResponse,
    routing::{get, post},
    Json, Router,
};
use chrono::Utc;
use futures::{SinkExt, StreamExt};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use uuid::Uuid;

use crate::clickhouse_client::ClickHouseClient;
use crate::models::*;
use crate::orbit_optimizer::{AlertManager, AtmosphericDragModel, GeneticOrbitOptimizer};
use crate::sgp4_engine::{CollisionAnalysis, CollisionProbabilityCalculator, Sgp4Propagator};

pub struct AppState {
    pub clickhouse: ClickHouseClient,
    pub propagator: Sgp4Propagator,
    pub calculator: CollisionProbabilityCalculator,
    pub optimizer: GeneticOrbitOptimizer,
    pub alert_manager: AlertManager,
    pub drag_model: AtmosphericDragModel,
    pub latest_telemetry: HashMap<u16, TelemetryData>,
    pub tle_cache: HashMap<u16, TleData>,
    pub active_analyses: Vec<CollisionAnalysis>,
    pub active_alerts: HashMap<Uuid, CollisionAlert>,
}

pub type SharedState = Arc<RwLock<AppState>>;

#[derive(Debug, Serialize)]
struct ApiError {
    error: String,
}

impl IntoResponse for ApiError {
    fn into_response(self) -> axum::response::Response {
        (axum::http::StatusCode::INTERNAL_SERVER_ERROR, Json(self)).into_response()
    }
}

pub fn create_router(state: SharedState) -> Router {
    Router::new()
        .route("/api/constellation/overview", get(constellation_overview))
        .route("/api/satellites", get(list_satellites))
        .route("/api/satellites/:id", get(get_satellite))
        .route("/api/satellites/:id/telemetry", get(get_telemetry_history))
        .route("/api/satellites/:id/propellant", get(get_propellant_history))
        .route("/api/satellites/:id/orbit-path", get(get_orbit_path))
        .route("/api/alerts", get(list_alerts))
        .route("/api/alerts/:id", get(get_alert))
        .route("/api/alerts/:id/acknowledge", post(acknowledge_alert))
        .route("/api/maneuvers", get(list_maneuvers))
        .route("/api/maneuvers/:id/execute", post(execute_maneuver))
        .route("/api/collision-analysis", get(list_collision_analysis))
        .route("/api/collision-encounters", get(list_collision_encounters))
        .route("/api/compute-avoidance/:alert_id", post(compute_avoidance))
        .route("/ws", get(websocket_handler))
        .with_state(state)
}

async fn constellation_overview(State(state): State<SharedState>) -> impl IntoResponse {
    let s = state.read().await;
    let total = s.latest_telemetry.len() as u32;
    let active_alerts = s.active_alerts.values().filter(|a| a.status == "active").count() as u32;
    let avg_propellant = if s.latest_telemetry.is_empty() {
        0.0
    } else {
        s.latest_telemetry.values().map(|t| t.propellant_remaining).sum::<f64>()
            / s.latest_telemetry.len() as f64
    };

    let coverage_status = if active_alerts == 0 {
        "nominal".to_string()
    } else if active_alerts < 5 {
        "degraded".to_string()
    } else {
        "critical".to_string()
    };

    Json(ConstellationOverview {
        total_satellites: total,
        active_alerts,
        avg_propellant,
        coverage_status,
    })
}

async fn list_satellites(State(state): State<SharedState>) -> impl IntoResponse {
    let s = state.read().await;
    let analyses_map: HashMap<u16, &CollisionAnalysis> = s
        .active_analyses
        .iter()
        .filter(|a| a.alert_level > 0)
        .flat_map(|a| {
            let mut m = Vec::new();
            if a.alert_level > 0 {
                m.push((a.satellite_id_1, a));
                m.push((a.satellite_id_2, a));
            }
            m
        })
        .fold(HashMap::new(), |mut acc, (id, analysis)| {
            acc.entry(id).and_modify(|e: &mut u8| {
                if analysis.alert_level > *e {
                    *e = analysis.alert_level;
                }
            }).or_insert(analysis.alert_level);
            acc
        });

    let mut max_level: HashMap<u16, u8> = HashMap::new();
    for analysis in &s.active_analyses {
        if analysis.alert_level > 0 {
            let e1 = max_level.entry(analysis.satellite_id_1).or_insert(0);
            *e1 = (*e1).max(analysis.alert_level);
            let e2 = max_level.entry(analysis.satellite_id_2).or_insert(0);
            *e2 = (*e2).max(analysis.alert_level);
        }
    }

    let satellites: Vec<SatelliteStatusResponse> = s
        .latest_telemetry
        .values()
        .map(|t| {
            let risk_level = match max_level.get(&t.satellite_id).copied().unwrap_or(0) {
                2 => CollisionRiskLevel::Danger,
                1 => CollisionRiskLevel::Warning,
                _ => CollisionRiskLevel::Safe,
            };

            let consumption_rate = 0.0;
            let est_lifetime = if consumption_rate > 0.0 {
                t.propellant_remaining / consumption_rate
            } else {
                t.propellant_remaining / 0.003 * 30.0 / 3600.0
            };

            SatelliteStatusResponse {
                satellite_id: t.satellite_id,
                name: format!("SAT-{:03}", t.satellite_id),
                current_position: Position3D {
                    x: t.position_x,
                    y: t.position_y,
                    z: t.position_z,
                },
                velocity: Velocity3D {
                    x: t.velocity_x,
                    y: t.velocity_y,
                    z: t.velocity_z,
                },
                orbital_elements: OrbitalElements {
                    semi_major_axis: t.semi_major_axis,
                    eccentricity: t.eccentricity,
                    inclination: t.inclination,
                    raan: t.raan,
                    arg_perigee: t.arg_perigee,
                    true_anomaly: t.true_anomaly,
                },
                propellant: PropellantInfo {
                    remaining: t.propellant_remaining,
                    consumption_rate,
                    estimated_lifetime_hours: est_lifetime,
                },
                collision_risk_level: risk_level,
            }
        })
        .collect();

    Json(satellites)
}

async fn get_satellite(
    State(state): State<SharedState>,
    Path(id): Path<u16>,
) -> Result<impl IntoResponse, Json<ApiError>> {
    let s = state.read().await;
    let t = s.latest_telemetry.get(&id).ok_or_else(|| {
        Json(ApiError {
            error: format!("Satellite {} not found", id),
        })
    })?;

    let max_alert = s.active_analyses.iter()
        .filter(|a| (a.satellite_id_1 == id || a.satellite_id_2 == id) && a.alert_level > 0)
        .map(|a| a.alert_level)
        .max()
        .unwrap_or(0);

    let risk_level = match max_alert {
        2 => CollisionRiskLevel::Danger,
        1 => CollisionRiskLevel::Warning,
        _ => CollisionRiskLevel::Safe,
    };

    let consumption_rate = 0.003 / 30.0 * 3600.0;
    let est_lifetime = t.propellant_remaining / consumption_rate.max(1e-10);

    Ok(Json(SatelliteStatusResponse {
        satellite_id: t.satellite_id,
        name: format!("SAT-{:03}", t.satellite_id),
        current_position: Position3D {
            x: t.position_x,
            y: t.position_y,
            z: t.position_z,
        },
        velocity: Velocity3D {
            x: t.velocity_x,
            y: t.velocity_y,
            z: t.velocity_z,
        },
        orbital_elements: OrbitalElements {
            semi_major_axis: t.semi_major_axis,
            eccentricity: t.eccentricity,
            inclination: t.inclination,
            raan: t.raan,
            arg_perigee: t.arg_perigee,
            true_anomaly: t.true_anomaly,
        },
        propellant: PropellantInfo {
            remaining: t.propellant_remaining,
            consumption_rate,
            estimated_lifetime_hours: est_lifetime,
        },
        collision_risk_level: risk_level,
    }))
}

#[derive(Deserialize)]
struct TelemetryQuery {
    hours: Option<u32>,
}

async fn get_telemetry_history(
    State(state): State<SharedState>,
    Path(id): Path<u16>,
    axum::extract::Query(query): axum::extract::Query<TelemetryQuery>,
) -> impl IntoResponse {
    let hours = query.hours.unwrap_or(1);
    let s = state.read().await;
    match s.clickhouse.get_telemetry_history(id, hours).await {
        Ok(data) => Json(data),
        Err(_) => Json(vec![]),
    }
}

#[derive(Deserialize)]
struct PropellantQuery {
    hours: Option<u32>,
}

async fn get_propellant_history(
    State(state): State<SharedState>,
    Path(id): Path<u16>,
    axum::extract::Query(query): axum::extract::Query<PropellantQuery>,
) -> impl IntoResponse {
    let hours = query.hours.unwrap_or(24);
    let s = state.read().await;
    match s.clickhouse.get_propellant_history(id, hours).await {
        Ok(data) => Json(data),
        Err(_) => Json(vec![]),
    }
}

async fn get_orbit_path(
    State(state): State<SharedState>,
    Path(id): Path<u16>,
) -> impl IntoResponse {
    let s = state.read().await;
    if let Some(tle) = s.tle_cache.get(&id) {
        let period_min = 1440.0 / tle.mean_motion;
        let step = period_min / 100.0;
        let states = s.propagator.propagate_batch(tle, 0.0, period_min, step);
        let positions: Vec<Position3D> = states
            .iter()
            .map(|s| Position3D {
                x: s.position_x,
                y: s.position_y,
                z: s.position_z,
            })
            .collect();
        Json(positions)
    } else {
        Json(vec![])
    }
}

async fn list_alerts(State(state): State<SharedState>) -> impl IntoResponse {
    let s = state.read().await;
    let alerts: Vec<CollisionAlertResponse> = s
        .active_alerts
        .values()
        .filter(|a| a.status == "active")
        .map(|a| CollisionAlertResponse {
            alert_id: a.alert_id,
            timestamp: a.timestamp,
            satellite_id_1: a.satellite_id_1,
            satellite_id_2: a.satellite_id_2,
            satellite_name_1: format!("SAT-{:03}", a.satellite_id_1),
            satellite_name_2: format!("SAT-{:03}", a.satellite_id_2),
            tca: a.tca,
            miss_distance: a.miss_distance,
            collision_probability: a.collision_probability,
            alert_level: a.alert_level,
            status: a.status.clone(),
            maneuver_planned: a.maneuver_planned,
        })
        .collect();
    Json(alerts)
}

async fn get_alert(
    State(state): State<SharedState>,
    Path(id): Path<String>,
) -> Result<impl IntoResponse, Json<ApiError>> {
    let alert_id = Uuid::parse_str(&id).map_err(|_| Json(ApiError {
        error: "Invalid alert ID".to_string(),
    }))?;
    let s = state.read().await;
    let alert = s.active_alerts.get(&alert_id).ok_or_else(|| Json(ApiError {
        error: "Alert not found".to_string(),
    }))?;
    Ok(Json(CollisionAlertResponse {
        alert_id: alert.alert_id,
        timestamp: alert.timestamp,
        satellite_id_1: alert.satellite_id_1,
        satellite_id_2: alert.satellite_id_2,
        satellite_name_1: format!("SAT-{:03}", alert.satellite_id_1),
        satellite_name_2: format!("SAT-{:03}", alert.satellite_id_2),
        tca: alert.tca,
        miss_distance: alert.miss_distance,
        collision_probability: alert.collision_probability,
        alert_level: alert.alert_level,
        status: alert.status.clone(),
        maneuver_planned: alert.maneuver_planned,
    }))
}

async fn acknowledge_alert(
    State(state): State<SharedState>,
    Path(id): Path<String>,
) -> Result<impl IntoResponse, Json<ApiError>> {
    let alert_id = Uuid::parse_str(&id).map_err(|_| Json(ApiError {
        error: "Invalid alert ID".to_string(),
    }))?;
    let mut s = state.write().await;
    if let Some(alert) = s.active_alerts.get_mut(&alert_id) {
        alert.status = "acknowledged".to_string();
        Ok(Json(serde_json::json!({"status": "acknowledged"})))
    } else {
        Err(Json(ApiError {
            error: "Alert not found".to_string(),
        }))
    }
}

async fn list_maneuvers(State(state): State<SharedState>) -> impl IntoResponse {
    Json(vec::<OrbitManeuverResponse>::new())
}

async fn execute_maneuver(
    State(_state): State<SharedState>,
    Path(_id): Path<String>,
) -> impl IntoResponse {
    Json(serde_json::json!({"status": "executed"}))
}

#[derive(Serialize)]
struct CollisionEncounter {
    satellite_id_1: u16,
    satellite_id_2: u16,
    encounter_point_eci: [f64; 3],
    collision_probability: f64,
    alert_level: u8,
}

async fn list_collision_analysis(State(state): State<SharedState>) -> impl IntoResponse {
    let s = state.read().await;
    Json(&s.active_analyses)
}

async fn list_collision_encounters(State(state): State<SharedState>) -> impl IntoResponse {
    let s = state.read().await;
    let encounters: Vec<CollisionEncounter> = s
        .active_analyses
        .iter()
        .filter(|a| a.alert_level > 0)
        .map(|a| CollisionEncounter {
            satellite_id_1: a.satellite_id_1,
            satellite_id_2: a.satellite_id_2,
            encounter_point_eci: [
                a.encounter_point_eci.0,
                a.encounter_point_eci.1,
                a.encounter_point_eci.2,
            ],
            collision_probability: a.collision_probability,
            alert_level: a.alert_level,
        })
        .collect();
    Json(encounters)
}

async fn compute_avoidance(
    State(state): State<SharedState>,
    Path(alert_id): Path<String>,
) -> Result<impl IntoResponse, Json<ApiError>> {
    let alert_uuid = Uuid::parse_str(&alert_id).map_err(|_| Json(ApiError {
        error: "Invalid alert ID".to_string(),
    }))?;

    let s = state.read().await;
    let alert = s.active_alerts.get(&alert_uuid).ok_or_else(|| Json(ApiError {
        error: "Alert not found".to_string(),
    }))?;

    let t1 = s.latest_telemetry.get(&alert.satellite_id_1).ok_or_else(|| Json(ApiError {
        error: "Satellite 1 telemetry not found".to_string(),
    }))?;
    let t2 = s.latest_telemetry.get(&alert.satellite_id_2).ok_or_else(|| Json(ApiError {
        error: "Satellite 2 telemetry not found".to_string(),
    }))?;
    let tle1 = s.tle_cache.get(&alert.satellite_id_1).ok_or_else(|| Json(ApiError {
        error: "Satellite 1 TLE not found".to_string(),
    }))?;
    let tle2 = s.tle_cache.get(&alert.satellite_id_2).ok_or_else(|| Json(ApiError {
        error: "Satellite 2 TLE not found".to_string(),
    }))?;

    let analysis = s.calculator.analyze_pair(&s.propagator, tle1, tle2, 72.0);

    let (_alert, maneuver) = s.alert_manager.compute_emergency_avoidance(
        &analysis,
        &s.optimizer,
        t1,
        t2,
        tle1,
        tle2,
        &s.propagator,
    );

    let response = OrbitManeuverResponse {
        maneuver_id: maneuver.maneuver_id,
        satellite_id: maneuver.satellite_id,
        timestamp: maneuver.timestamp,
        maneuver_type: maneuver.maneuver_type,
        delta_v_x: maneuver.delta_v_x,
        delta_v_y: maneuver.delta_v_y,
        delta_v_z: maneuver.delta_v_z,
        fuel_cost: maneuver.fuel_cost,
        target_semi_major_axis: maneuver.target_semi_major_axis,
        target_inclination: maneuver.target_inclination,
        executed: maneuver.executed,
    };

    Ok(Json(response))
}

async fn websocket_handler(ws: WebSocketUpgrade, State(state): State<SharedState>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_websocket(socket, state))
}

async fn handle_websocket(socket: WebSocket, state: SharedState) {
    let (mut sender, mut receiver) = socket.split();

    let mut interval = tokio::time::interval(std::time::Duration::from_secs(1));
    let state_clone = state.clone();

    let send_task = tokio::spawn(async move {
        loop {
            interval.tick().await;
            let s = state_clone.read().await;
            let positions: Vec<serde_json::Value> = s
                .latest_telemetry
                .values()
                .map(|t| {
                    serde_json::json!({
                        "satellite_id": t.satellite_id,
                        "position": {
                            "x": t.position_x,
                            "y": t.position_y,
                            "z": t.position_z,
                        },
                        "velocity": {
                            "x": t.velocity_x,
                            "y": t.velocity_y,
                            "z": t.velocity_z,
                        }
                    })
                })
                .collect();
            let msg = serde_json::json!(positions);
            if sender.send(Message::Text(msg.to_string())).await.is_err() {
                break;
            }
        }
    });

    let recv_task = tokio::spawn(async move {
        while let Some(Ok(msg)) = receiver.next().await {
            if matches!(msg, Message::Close(_)) {
                break;
            }
        }
    });

    tokio::select! {
        _ = send_task => {},
        _ = recv_task => {},
    }
}
