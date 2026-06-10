use satellite_constellation_system::{
    api::{AppState, SharedState, create_router},
    clickhouse_client::ClickHouseClient,
    models::*,
    orbit_optimizer::{AlertManager, AtmosphericDragModel, GeneticOrbitOptimizer},
    sgp4_engine::{CollisionProbabilityCalculator, NumericalPropagator, NumericalPropagatorConfig, Sgp4Propagator},
    udp_receiver::start_udp_receiver,
};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::{mpsc, RwLock};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info".into()),
        )
        .init();

    tracing::info!("Starting Satellite Constellation Orbit Control & Collision Warning System");

    let ch_url = std::env::var("CLICKHOUSE_URL").unwrap_or_else(|_| "http://localhost:8123".to_string());
    let ch_db = "satellite_constellation";

    let clickhouse = ClickHouseClient::new(&ch_url, ch_db);
    tracing::info!("ClickHouse client initialized: {}/{}", ch_url, ch_db);

    let state: SharedState = Arc::new(RwLock::new(AppState {
        clickhouse,
        propagator: Sgp4Propagator::new(),
        numerical_propagator: NumericalPropagator::new(NumericalPropagatorConfig::default()),
        calculator: CollisionProbabilityCalculator::new(),
        optimizer: GeneticOrbitOptimizer::new(50, 30, 0.15),
        alert_manager: AlertManager::new(),
        drag_model: AtmosphericDragModel::new(),
        latest_telemetry: HashMap::new(),
        tle_cache: HashMap::new(),
        active_analyses: Vec::new(),
        active_alerts: HashMap::new(),
    }));

    let (tx, mut rx) = mpsc::channel::<TelemetryData>(10000);

    let udp_state = state.clone();
    let udp_task = tokio::spawn(async move {
        match start_udp_receiver(tx).await {
            Ok(_) => tracing::info!("UDP receiver stopped"),
            Err(e) => tracing::error!("UDP receiver error: {}", e),
        }
    });

    let telemetry_state = state.clone();
    let telemetry_task = tokio::spawn(async move {
        while let Some(data) = rx.recv().await {
            let mut s = telemetry_state.write().await;

            s.latest_telemetry.insert(data.satellite_id, data.clone());

            if let Err(e) = s.clickhouse.insert_telemetry(&data).await {
                tracing::warn!("Failed to insert telemetry for sat {}: {}", data.satellite_id, e);
            }
        }
    });

    let collision_state = state.clone();
    let collision_task = tokio::spawn(async move {
        let mut interval = tokio::time::interval(std::time::Duration::from_secs(300));
        loop {
            interval.tick().await;
            let mut s = collision_state.write().await;

            if s.tle_cache.is_empty() {
                tracing::debug!("TLE cache empty, skipping collision analysis");
                continue;
            }

            let tle_ids: Vec<u16> = s.tle_cache.keys().copied().collect();
            let tle_count = tle_ids.len();
            tracing::info!("Running collision analysis for {} TLE entries", tle_count);

            let mut analyses = Vec::new();

            for i in 0..tle_ids.len() {
                for j in (i + 1)..tle_ids.len() {
                    let id1 = tle_ids[i];
                    let id2 = tle_ids[j];

                    if let (Some(tle1), Some(tle2)) = (s.tle_cache.get(&id1), s.tle_cache.get(&id2)) {
                        let analysis = s.calculator.analyze_pair_dual(
                            &s.propagator,
                            &s.numerical_propagator,
                            tle1,
                            tle2,
                            72.0,
                        );

                        if analysis.alert_level > 0 {
                            tracing::warn!(
                                "Collision risk: SAT-{:03} vs SAT-{:03}, prob={:.2e}, level={}, miss={:.3}km",
                                id1, id2, analysis.collision_probability, analysis.alert_level,
                                analysis.tca_result.miss_distance
                            );

                            if let Some(alert) = s.alert_manager.evaluate_collision(&analysis) {
                                if s.active_alerts.insert(alert.alert_id, alert.clone()).is_none() {
                                    let alert_clone = alert.clone();
                                    let alert_state = collision_state.clone();
                                    tokio::spawn(async move {
                                        let s = alert_state.read().await;
                                        if let Err(e) = s.alert_manager.push_alert_to_ground_station(&alert_clone).await {
                                            tracing::warn!("Failed to push alert to ground station: {}", e);
                                        }
                                    });
                                }
                            }

                            if analysis.alert_level == 2 {
                                if let (Some(t1), Some(t2)) =
                                    (s.latest_telemetry.get(&id1), s.latest_telemetry.get(&id2))
                                {
                                    if let (Some(tle1_ref), Some(tle2_ref)) =
                                        (s.tle_cache.get(&id1), s.tle_cache.get(&id2))
                                    {
                                        let (_alert, maneuver) = s.alert_manager.compute_emergency_avoidance(
                                            &analysis,
                                            &s.optimizer,
                                            t1,
                                            t2,
                                            tle1_ref,
                                            tle2_ref,
                                            &s.propagator,
                                        );

                                        if let Err(e) = s.clickhouse.insert_orbit_maneuver(&maneuver).await {
                                            tracing::warn!("Failed to insert avoidance maneuver: {}", e);
                                        }

                                        let maneuver_clone = maneuver.clone();
                                        let alert_state = collision_state.clone();
                                        tokio::spawn(async move {
                                            let s = alert_state.read().await;
                                            if let Err(e) = s.alert_manager.push_maneuver_to_ground_station(&maneuver_clone).await {
                                                tracing::warn!("Failed to push maneuver to ground station: {}", e);
                                            }
                                        });
                                    }
                                }
                            }
                        }

                        analyses.push(analysis);
                    }
                }
            }

            s.active_analyses = analyses;
            tracing::info!("Collision analysis complete");
        }
    });

    let propellant_state = state.clone();
    let propellant_task = tokio::spawn(async move {
        let mut interval = tokio::time::interval(std::time::Duration::from_secs(60));
        loop {
            interval.tick().await;
            let s = propellant_state.read().await;

            for (id, telemetry) in &s.latest_telemetry {
                let consumption_rate = 0.003 / 30.0 * 3600.0;
                let est_lifetime = if consumption_rate > 0.0 {
                    telemetry.propellant_remaining / consumption_rate
                } else {
                    999999.0
                };

                let history = PropellantHistory {
                    satellite_id: *id,
                    timestamp: chrono::Utc::now(),
                    propellant_remaining: telemetry.propellant_remaining,
                    consumption_rate,
                    estimated_lifetime_hours: est_lifetime,
                };

                if let Err(e) = s.clickhouse.insert_propellant_history(&history).await {
                    tracing::debug!("Failed to insert propellant history for sat {}: {}", id, e);
                }
            }
        }
    });

    let tle_update_state = state.clone();
    let tle_task = tokio::spawn(async move {
        let udp_socket = tokio::net::UdpSocket::bind("0.0.0.0:9091").await?;
        tracing::info!("TLE receiver listening on 0.0.0.0:9091");
        let mut buf = [0u8; 8192];
        loop {
            match udp_socket.recv_from(&mut buf).await {
                Ok((len, _addr)) => {
                    let json_str = String::from_utf8_lossy(&buf[..len]);
                    match serde_json::from_str::<TleData>(&json_str) {
                        Ok(tle) => {
                            let mut s = tle_update_state.write().await;
                            s.tle_cache.insert(tle.satellite_id, tle);
                        }
                        Err(e) => {
                            tracing::debug!("TLE parse error: {}", e);
                        }
                    }
                }
                Err(e) => {
                    tracing::warn!("TLE receive error: {}", e);
                }
            }
        }
        #[allow(unreachable_code)]
        Ok::<(), anyhow::Error>(())
    });

    let app = create_router(state.clone())
        .layer(tower_http::cors::CorsLayer::permissive())
        .fallback(static_files_fallback);

    let listener = tokio::net::TcpListener::bind("0.0.0.0:8080").await?;
    tracing::info!("HTTP server listening on 0.0.0.0:8080");

    let server = axum::serve(listener, app)
        .with_graceful_shutdown(shutdown_signal());

    if let Err(e) = server.await {
        tracing::error!("Server error: {}", e);
    }

    udp_task.abort();
    telemetry_task.abort();
    collision_task.abort();
    propellant_task.abort();
    tle_task.abort();

    tracing::info!("System shutdown complete");
    Ok(())
}

async fn static_files_fallback() -> impl IntoResponse {
    (
        axum::http::StatusCode::OK,
        axum::http::HeaderMap::new(),
        include_str!("../../frontend/index.html"),
    )
}

async fn shutdown_signal() {
    tokio::signal::ctrl_c()
        .await
        .expect("Failed to install CTRL+C handler");
    tracing::info!("Shutdown signal received");
}
