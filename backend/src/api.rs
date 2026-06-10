use axum::{
    extract::{
        Path, State, WebSocketUpgrade, ws::{Message, WebSocket},
    },
    response::IntoResponse,
    routing::{get, post},
    Json, Router,
};
use futures::{SinkExt, StreamExt};
use lazy_static::lazy_static;
use prometheus::{IntCounter, IntGauge, Histogram, Registry, Encoder, TextEncoder};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::{mpsc, oneshot, RwLock};
use uuid::Uuid;

use crate::clickhouse_client::ClickHouseClient;
use crate::collision_predictor::{CollisionAnalysis, Sgp4Propagator};
use crate::config::AppConfig;
use crate::models::*;
use crate::orbit_optimizer_service::{OptimizerRequest, ManeuverPlan};

lazy_static! {
    pub static ref REGISTRY: Registry = Registry::new();
    pub static ref TELEMETRY_RECEIVED: IntCounter = IntCounter::new(
        "telemetry_received_total", "Total telemetry packets received"
    ).unwrap();
    pub static ref ACTIVE_SATELLITES: IntGauge = IntGauge::new(
        "active_satellites", "Number of satellites with recent telemetry"
    ).unwrap();
    pub static ref ACTIVE_ALERTS: IntGauge = IntGauge::new(
        "active_alerts", "Number of active collision alerts"
    ).unwrap();
    pub static ref COLLISION_ANALYSIS_SECONDS: Histogram = Histogram::with_opts(
        HistogramOpts::new("collision_analysis_duration_seconds", "Time spent on collision analysis cycle")
    ).unwrap();
    pub static ref AVOIDANCE_COMPUTATIONS: IntCounter = IntCounter::new(
        "avoidance_computations_total", "Total avoidance maneuver computations"
    ).unwrap();
    pub static ref HTTP_REQUESTS: IntCounter = IntCounter::new(
        "http_requests_total", "Total HTTP requests served"
    ).unwrap();
}

pub fn init_metrics() {
    REGISTRY.register(Box::new(TELEMETRY_RECEIVED.clone())).unwrap();
    REGISTRY.register(Box::new(ACTIVE_SATELLITES.clone())).unwrap();
    REGISTRY.register(Box::new(ACTIVE_ALERTS.clone())).unwrap();
    REGISTRY.register(Box::new(COLLISION_ANALYSIS_SECONDS.clone())).unwrap();
    REGISTRY.register(Box::new(AVOIDANCE_COMPUTATIONS.clone())).unwrap();
    REGISTRY.register(Box::new(HTTP_REQUESTS.clone())).unwrap();
}

pub struct AppState {
    pub clickhouse: ClickHouseClient,
    pub latest_telemetry: Arc<RwLock<HashMap<u16, TelemetryData>>>,
    pub tle_cache: Arc<RwLock<HashMap<u16, TleData>>>,
    pub optimizer_request_tx: mpsc::Sender<OptimizerRequest>,
    pub config: AppConfig,
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
        .route("/metrics", get(metrics_handler))
        .route("/ws", get(websocket_handler))
        .with_state(state)
}

async fn constellation_overview(State(state): State<SharedState>) -> impl IntoResponse {
    HTTP_REQUESTS.inc();
    let s = state.read().await;
    let tel_map = s.latest_telemetry.read().await;
    let total = tel_map.len() as u32;
    let avg_propellant = if tel_map.is_empty() {
        0.0
    } else {
        tel_map.values().map(|t| t.propellant_remaining).sum::<f64>() / tel_map.len() as f64
    };
    let coverage_status = if avg_propellant > 30.0 {
        "nominal".to_string()
    } else if avg_propellant > 15.0 {
        "degraded".to_string()
    } else {
        "critical".to_string()
    };
    drop(tel_map);

    let active_alerts = s.clickhouse.get_active_alerts()
        .await
        .map(|a| a.len() as u32)
        .unwrap_or(0);

    ACTIVE_SATELLITES.set(total as i64);
    ACTIVE_ALERTS.set(active_alerts as i64);

    Json(ConstellationOverview {
        total_satellites: total,
        active_alerts,
        avg_propellant,
        coverage_status,
    })
}

async fn list_satellites(State(state): State<SharedState>) -> impl IntoResponse {
    let s = state.read().await;
    let tel_map = s.latest_telemetry.read().await;
    let tle_map = s.tle_cache.read().await;

    let analyses_map: HashMap<u16, u8> = HashMap::new();
    let _ = &analyses_map;

    let satellites: Vec<SatelliteStatusResponse> = tel_map
        .values()
        .map(|t| {
            let risk_level = CollisionRiskLevel::Safe;

            let est_lifetime = t.propellant_remaining / 0.003 * 30.0 / 3600.0;

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
                    consumption_rate: 0.003 / 30.0 * 3600.0,
                    estimated_lifetime_hours: est_lifetime,
                },
                collision_risk_level: risk_level,
            }
        })
        .collect();
    drop(tle_map);
    drop(tel_map);

    Json(satellites)
}

async fn get_satellite(
    State(state): State<SharedState>,
    Path(id): Path<u16>,
) -> Result<impl IntoResponse, Json<ApiError>> {
    let s = state.read().await;
    let tel_map = s.latest_telemetry.read().await;
    let t = tel_map.get(&id).ok_or_else(|| {
        Json(ApiError {
            error: format!("Satellite {} not found", id),
        })
    })?;
    let t = t.clone();
    drop(tel_map);

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
        collision_risk_level: CollisionRiskLevel::Safe,
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
    let tle_map = s.tle_cache.read().await;
    if let Some(tle) = tle_map.get(&id) {
        let propagator = Sgp4Propagator::new(&s.config.sgp4);
        let period_min = 1440.0 / tle.mean_motion;
        let step = period_min / 100.0;
        let states = propagator.propagate_batch(tle, 0.0, period_min, step);
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
    match s.clickhouse.get_active_alerts().await {
        Ok(alerts) => {
            let responses: Vec<CollisionAlertResponse> = alerts
                .iter()
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
            Json(responses)
        }
        Err(_) => Json(vec![]),
    }
}

async fn get_alert(
    State(state): State<SharedState>,
    Path(id): Path<String>,
) -> Result<impl IntoResponse, Json<ApiError>> {
    let alert_id = Uuid::parse_str(&id).map_err(|_| Json(ApiError {
        error: "Invalid alert ID".to_string(),
    }))?;
    let s = state.read().await;
    match s.clickhouse.get_active_alerts().await {
        Ok(alerts) => {
            let alert = alerts.into_iter().find(|a| a.alert_id == alert_id).ok_or_else(|| Json(ApiError {
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
                status: alert.status,
                maneuver_planned: alert.maneuver_planned,
            }))
        }
        Err(_) => Err(Json(ApiError {
            error: "Alert not found".to_string(),
        })),
    }
}

async fn acknowledge_alert(
    State(_state): State<SharedState>,
    Path(_id): Path<String>,
) -> Result<impl IntoResponse, Json<ApiError>> {
    Ok(Json(serde_json::json!({"status": "acknowledged"})))
}

async fn list_maneuvers(State(_state): State<SharedState>) -> impl IntoResponse {
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
    match s.clickhouse.get_active_alerts().await {
        Ok(_alerts) => Json(vec::<CollisionAnalysis>::new()),
        Err(_) => Json(vec::<CollisionAnalysis>::new()),
    }
}

async fn list_collision_encounters(State(state): State<SharedState>) -> impl IntoResponse {
    let s = state.read().await;
    match s.clickhouse.get_active_alerts().await {
        Ok(alerts) => {
            let encounters: Vec<CollisionEncounter> = alerts
                .iter()
                .filter(|a| a.alert_level > 0)
                .map(|a| CollisionEncounter {
                    satellite_id_1: a.satellite_id_1,
                    satellite_id_2: a.satellite_id_2,
                    encounter_point_eci: [0.0, 0.0, 0.0],
                    collision_probability: a.collision_probability,
                    alert_level: a.alert_level,
                })
                .collect();
            Json(encounters)
        }
        Err(_) => Json(vec![]),
    }
}

async fn compute_avoidance(
    State(state): State<SharedState>,
    Path(alert_id): Path<String>,
) -> Result<impl IntoResponse, Json<ApiError>> {
    let alert_uuid = Uuid::parse_str(&alert_id).map_err(|_| Json(ApiError {
        error: "Invalid alert ID".to_string(),
    }))?;

    let s = state.read().await;
    let alerts = s.clickhouse.get_active_alerts().await.map_err(|_| Json(ApiError {
        error: "Failed to query alerts".to_string(),
    }))?;
    let alert = alerts.into_iter().find(|a| a.alert_id == alert_uuid).ok_or_else(|| Json(ApiError {
        error: "Alert not found".to_string(),
    }))?;

    let tel_map = s.latest_telemetry.read().await;
    let tle_map = s.tle_cache.read().await;
    let t1 = tel_map.get(&alert.satellite_id_1).ok_or_else(|| Json(ApiError {
        error: "Satellite 1 telemetry not found".to_string(),
    }))?;
    let tle1 = tle_map.get(&alert.satellite_id_1).ok_or_else(|| Json(ApiError {
        error: "Satellite 1 TLE not found".to_string(),
    }))?;
    let tle2 = tle_map.get(&alert.satellite_id_2).ok_or_else(|| Json(ApiError {
        error: "Satellite 2 TLE not found".to_string(),
    }))?;

    let (reply_tx, reply_rx) = oneshot::channel();
    let request = OptimizerRequest::AvoidanceManeuver {
        telemetry: t1.clone(),
        tle1: tle1.clone(),
        tle2: tle2.clone(),
        reply: reply_tx,
    };

    drop(tel_map);
    drop(tle_map);

    s.optimizer_request_tx.send(request).await.map_err(|_| Json(ApiError {
        error: "Optimizer service unavailable".to_string(),
    }))?;

    let plan = reply_rx.await.map_err(|_| Json(ApiError {
        error: "Optimizer request failed".to_string(),
    }))?;

    let maneuver = OrbitManeuver {
        maneuver_id: Uuid::new_v4(),
        satellite_id: plan.satellite_id,
        timestamp: chrono::Utc::now(),
        maneuver_type: "collision_avoidance".to_string(),
        delta_v_x: plan.delta_v_x,
        delta_v_y: plan.delta_v_y,
        delta_v_z: plan.delta_v_z,
        fuel_cost: plan.fuel_cost,
        target_semi_major_axis: plan.target_semi_major_axis,
        target_inclination: 0.0,
        executed: false,
    };

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

    AVOIDANCE_COMPUTATIONS.inc();

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
            let tel_map = s.latest_telemetry.read().await;
            let positions: Vec<serde_json::Value> = tel_map
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
            drop(tel_map);
            drop(s);
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

async fn metrics_handler() -> impl IntoResponse {
    let encoder = TextEncoder::new();
    let metric_families = REGISTRY.gather();
    let mut buffer = Vec::new();
    encoder.encode(&metric_families, &mut buffer).unwrap();
    let body = String::from_utf8(buffer).unwrap_or_default();
    (
        [(axum::http::header::CONTENT_TYPE, "text/plain; version=0.0.4")],
        body,
    )
}
