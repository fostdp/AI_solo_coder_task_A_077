use chrono::Utc;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::{mpsc, oneshot, RwLock};
use uuid::Uuid;

use crate::clickhouse_client::ClickHouseClient;
use crate::collision_predictor::CollisionAnalysis;
use crate::config::GroundStationConfig;
use crate::models::{CollisionAlert, OrbitManeuver, TelemetryData, TleData};
use crate::orbit_optimizer_service::{AlertManager, OptimizerRequest};

pub struct AlarmCommander {
    alert_manager: AlertManager,
    clickhouse: ClickHouseClient,
    active_alerts: HashMap<Uuid, CollisionAlert>,
    active_analyses: Vec<CollisionAnalysis>,
    optimizer_request_tx: mpsc::Sender<OptimizerRequest>,
    latest_telemetry: Arc<RwLock<HashMap<u16, TelemetryData>>>,
    tle_cache: Arc<RwLock<HashMap<u16, TleData>>>,
}

impl AlarmCommander {
    pub fn new(
        alert_manager: AlertManager,
        clickhouse: ClickHouseClient,
        optimizer_request_tx: mpsc::Sender<OptimizerRequest>,
        latest_telemetry: Arc<RwLock<HashMap<u16, TelemetryData>>>,
        tle_cache: Arc<RwLock<HashMap<u16, TleData>>>,
    ) -> Self {
        Self {
            alert_manager,
            clickhouse,
            active_alerts: HashMap::new(),
            active_analyses: Vec::new(),
            optimizer_request_tx,
            latest_telemetry,
            tle_cache,
        }
    }

    pub async fn run(mut self, mut analysis_rx: mpsc::Receiver<CollisionAnalysis>) {
        let mut cycle_analyses: Vec<CollisionAnalysis> = Vec::new();

        while let Some(analysis) = analysis_rx.recv().await {
            cycle_analyses.push(analysis.clone());

            if analysis.alert_level > 0 {
                if let Some(alert) = self.alert_manager.evaluate_collision(&analysis) {
                    let is_new = self.active_alerts.insert(alert.alert_id, alert.clone()).is_none();

                    if is_new {
                        if let Err(e) = self.clickhouse.insert_collision_alert(&alert).await {
                            tracing::warn!("Failed to insert alert to ClickHouse: {}", e);
                        }

                        let alert_clone = alert.clone();
                        let alert_mgr = AlertManager::new(GroundStationConfig {
                            alert_url: String::new(),
                            maneuver_url: String::new(),
                            push_timeout_seconds: 5,
                        });
                        tokio::spawn(async move {
                            let cfg = GroundStationConfig {
                                alert_url: "http://localhost:8888/ground-station/alert".to_string(),
                                maneuver_url: String::new(),
                                push_timeout_seconds: 5,
                            };
                            let mgr = AlertManager::new(cfg);
                            if let Err(e) = mgr.push_alert_to_ground_station(&alert_clone).await {
                                tracing::warn!("Failed to push alert to ground station: {}", e);
                            }
                        });
                    }
                }

                if analysis.alert_level == 2 {
                    let id1 = analysis.satellite_id_1;
                    let id2 = analysis.satellite_id_2;

                    let tel = self.latest_telemetry.read().await;
                    let tle = self.tle_cache.read().await;

                    if let (Some(t1), Some(tle1)) = (tel.get(&id1), tle.get(&id1)) {
                        if let (Some(_t2), Some(tle2)) = (tel.get(&id2), tle.get(&id2)) {
                            let (reply_tx, reply_rx) = oneshot::channel();

                            let request = OptimizerRequest::AvoidanceManeuver {
                                telemetry: t1.clone(),
                                tle1: tle1.clone(),
                                tle2: tle2.clone(),
                                reply: reply_tx,
                            };

                            if self.optimizer_request_tx.send(request).await.is_ok() {
                                if let Ok(plan) = reply_rx.await {
                                    let maneuver = OrbitManeuver {
                                        maneuver_id: Uuid::new_v4(),
                                        satellite_id: t1.satellite_id,
                                        timestamp: Utc::now(),
                                        maneuver_type: "collision_avoidance".to_string(),
                                        delta_v_x: plan.delta_v_x,
                                        delta_v_y: plan.delta_v_y,
                                        delta_v_z: plan.delta_v_z,
                                        fuel_cost: plan.fuel_cost,
                                        target_semi_major_axis: t1.semi_major_axis,
                                        target_inclination: t1.inclination,
                                        executed: false,
                                    };

                                    if let Err(e) = self.clickhouse.insert_orbit_maneuver(&maneuver).await {
                                        tracing::warn!("Failed to insert avoidance maneuver: {}", e);
                                    }

                                    let maneuver_clone = maneuver.clone();
                                    tokio::spawn(async move {
                                        let cfg = GroundStationConfig {
                                            alert_url: String::new(),
                                            maneuver_url: "http://localhost:8888/ground-station/maneuver".to_string(),
                                            push_timeout_seconds: 5,
                                        };
                                        let mgr = AlertManager::new(cfg);
                                        if let Err(e) = mgr.push_maneuver_to_ground_station(&maneuver_clone).await {
                                            tracing::warn!("Failed to push maneuver to ground station: {}", e);
                                        }
                                    });
                                }
                            }
                        }
                    }
                }
            }
        }

        self.active_analyses = cycle_analyses;
    }

    pub fn get_active_alerts(&self) -> &HashMap<Uuid, CollisionAlert> {
        &self.active_alerts
    }

    pub fn get_active_analyses(&self) -> &[CollisionAnalysis] {
        &self.active_analyses
    }
}
