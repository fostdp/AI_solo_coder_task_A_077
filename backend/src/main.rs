use satellite_constellation_system::{
    alarm_commander::AlarmCommander,
    api::{AppState, SharedState, create_router, init_metrics, TELEMETRY_RECEIVED},
    clickhouse_client::ClickHouseClient,
    collision_predictor::{CollisionAnalysis, CollisionPredictor},
    config::AppConfig,
    constellation_receiver::ConstellationReceiver,
    models::*,
    orbit_optimizer_service::{AlertManager, OrbitOptimizerService, OptimizerRequest},
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

    init_metrics();

    let config = AppConfig::load()?;
    tracing::info!("Configuration loaded from config.toml");

    let clickhouse = ClickHouseClient::new(
        &config.network.clickhouse_url,
        &config.network.clickhouse_database,
    );
    tracing::info!(
        "ClickHouse client initialized: {}/{}",
        config.network.clickhouse_url,
        config.network.clickhouse_database
    );

    let latest_telemetry: Arc<RwLock<HashMap<u16, TelemetryData>>> =
        Arc::new(RwLock::new(HashMap::new()));
    let tle_cache: Arc<RwLock<HashMap<u16, TleData>>> =
        Arc::new(RwLock::new(HashMap::new()));

    let (raw_telemetry_tx, raw_telemetry_rx) = mpsc::channel::<TelemetryData>(10000);
    let (raw_tle_tx, raw_tle_rx) = mpsc::channel::<TleData>(1000);
    let (cp_telemetry_tx, cp_telemetry_rx) = mpsc::channel::<TelemetryData>(10000);
    let (cp_tle_tx, cp_tle_rx) = mpsc::channel::<TleData>(1000);
    let (analysis_tx, analysis_rx) = mpsc::channel::<CollisionAnalysis>(5000);
    let (optimizer_request_tx, optimizer_request_rx) = mpsc::channel::<OptimizerRequest>(100);

    let receiver = ConstellationReceiver::new(
        config.network.telemetry_udp_port,
        config.network.tle_udp_port,
        config.reorder_buffer.clone(),
    );
    let udp_task = tokio::spawn(async move {
        if let Err(e) = receiver.run(raw_telemetry_tx, raw_tle_tx).await {
            tracing::error!("Constellation receiver error: {}", e);
        }
    });

    let fanout_tel_state = latest_telemetry.clone();
    let fanout_tel_ch = clickhouse.clone();
    let fanout_tel_cp = cp_telemetry_tx;
    let telemetry_task = tokio::spawn(async move {
        while let Some(data) = raw_telemetry_rx.recv().await {
            TELEMETRY_RECEIVED.inc();
            {
                let mut map = fanout_tel_state.write().await;
                map.insert(data.satellite_id, data.clone());
            }
            if let Err(e) = fanout_tel_ch.insert_telemetry(&data).await {
                tracing::debug!("Failed to insert telemetry for sat {}: {}", data.satellite_id, e);
            }
            let _ = fanout_tel_cp.send(data).await;
        }
    });

    let fanout_tle_state = tle_cache.clone();
    let fanout_tle_cp = cp_tle_tx;
    let tle_task = tokio::spawn(async move {
        while let Some(tle) = raw_tle_rx.recv().await {
            {
                let mut map = fanout_tle_state.write().await;
                map.insert(tle.satellite_id, tle.clone());
            }
            let _ = fanout_tle_cp.send(tle).await;
        }
    });

    let collision_predictor = CollisionPredictor::new(&config);
    let collision_task = tokio::spawn(async move {
        collision_predictor.run(cp_telemetry_rx, cp_tle_rx, analysis_tx).await;
    });

    let optimizer_service = OrbitOptimizerService::new(&config);
    let optimizer_task = tokio::spawn(async move {
        optimizer_service.run(optimizer_request_rx).await;
    });

    let alert_manager = AlertManager::new(config.ground_station.clone());
    let alarm_commander = AlarmCommander::new(
        alert_manager,
        clickhouse.clone(),
        optimizer_request_tx.clone(),
        latest_telemetry.clone(),
        tle_cache.clone(),
    );
    let alarm_task = tokio::spawn(async move {
        alarm_commander.run(analysis_rx).await;
    });

    let propellant_state = latest_telemetry.clone();
    let propellant_ch = clickhouse.clone();
    let propellant_task = tokio::spawn(async move {
        let mut interval = tokio::time::interval(std::time::Duration::from_secs(60));
        loop {
            interval.tick().await;
            let tel_map = propellant_state.read().await;
            for (id, telemetry) in tel_map.iter() {
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
                if let Err(e) = propellant_ch.insert_propellant_history(&history).await {
                    tracing::debug!("Failed to insert propellant history for sat {}: {}", id, e);
                }
            }
        }
    });

    let state: SharedState = Arc::new(RwLock::new(AppState {
        clickhouse,
        latest_telemetry,
        tle_cache,
        optimizer_request_tx,
        config: config.clone(),
    }));

    let app = create_router(state.clone())
        .layer(tower_http::compression::CompressionLayer::new())
        .layer(tower_http::trace::TraceLayer::new_for_http())
        .layer(tower_http::cors::CorsLayer::permissive())
        .fallback(static_files_fallback);

    let listener = tokio::net::TcpListener::bind(format!(
        "0.0.0.0:{}",
        config.network.http_port
    ))
    .await?;
    tracing::info!("HTTP server listening on 0.0.0.0:{}", config.network.http_port);

    let server = axum::serve(listener, app).with_graceful_shutdown(shutdown_signal());

    if let Err(e) = server.await {
        tracing::error!("Server error: {}", e);
    }

    udp_task.abort();
    telemetry_task.abort();
    tle_task.abort();
    collision_task.abort();
    optimizer_task.abort();
    alarm_task.abort();
    propellant_task.abort();

    tracing::info!("System shutdown complete");
    Ok(())
}

async fn static_files_fallback() -> impl axum::response::IntoResponse {
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
