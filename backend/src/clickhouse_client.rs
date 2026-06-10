use anyhow::Result;
use clickhouse::Client;

use crate::models::{CollisionAlert, OrbitManeuver, PropellantHistory, TelemetryData, TleData};

#[derive(Clone)]
pub struct ClickHouseClient {
    client: Client,
}

impl ClickHouseClient {
    pub fn new(url: &str, database: &str) -> Self {
        let client = Client::default()
            .with_url(url)
            .with_database(database);
        Self { client }
    }

    pub async fn insert_telemetry(&self, data: &TelemetryData) -> Result<()> {
        let mut insert = self.client.insert::<TelemetryData>("telemetry")?;
        insert.write(data).await?;
        insert.end().await?;
        Ok(())
    }

    pub async fn insert_telemetry_batch(&self, data: &[TelemetryData]) -> Result<()> {
        let mut insert = self.client.insert::<TelemetryData>("telemetry")?;
        for item in data {
            insert.write(item).await?;
        }
        insert.end().await?;
        Ok(())
    }

    pub async fn insert_tle(&self, data: &TleData) -> Result<()> {
        let mut insert = self.client.insert::<TleData>("tle_data")?;
        insert.write(data).await?;
        insert.end().await?;
        Ok(())
    }

    pub async fn insert_collision_alert(&self, alert: &CollisionAlert) -> Result<()> {
        let mut insert = self.client.insert::<CollisionAlert>("collision_alerts")?;
        insert.write(alert).await?;
        insert.end().await?;
        Ok(())
    }

    pub async fn insert_orbit_maneuver(&self, maneuver: &OrbitManeuver) -> Result<()> {
        let mut insert = self.client.insert::<OrbitManeuver>("orbit_maneuvers")?;
        insert.write(maneuver).await?;
        insert.end().await?;
        Ok(())
    }

    pub async fn insert_propellant_history(&self, data: &PropellantHistory) -> Result<()> {
        let mut insert = self.client.insert::<PropellantHistory>("propellant_history")?;
        insert.write(data).await?;
        insert.end().await?;
        Ok(())
    }

    pub async fn get_latest_telemetry(&self, satellite_id: u16) -> Result<Option<TelemetryData>> {
        let rows = self.client
            .query("SELECT ?fields FROM telemetry WHERE satellite_id = ? ORDER BY timestamp DESC LIMIT 1")
            .bind(satellite_id)
            .fetch_all::<TelemetryData>()
            .await?;
        Ok(rows.into_iter().next())
    }

    pub async fn get_telemetry_history(&self, satellite_id: u16, hours: u32) -> Result<Vec<TelemetryData>> {
        let rows = self.client
            .query("SELECT ?fields FROM telemetry WHERE satellite_id = ? AND timestamp >= now() - INTERVAL ? HOUR ORDER BY timestamp")
            .bind(satellite_id)
            .bind(hours)
            .fetch_all::<TelemetryData>()
            .await?;
        Ok(rows)
    }

    pub async fn get_propellant_history(&self, satellite_id: u16, hours: u32) -> Result<Vec<PropellantHistory>> {
        let rows = self.client
            .query("SELECT ?fields FROM propellant_history WHERE satellite_id = ? AND timestamp >= now() - INTERVAL ? HOUR ORDER BY timestamp")
            .bind(satellite_id)
            .bind(hours)
            .fetch_all::<PropellantHistory>()
            .await?;
        Ok(rows)
    }

    pub async fn get_active_alerts(&self) -> Result<Vec<CollisionAlert>> {
        let rows = self.client
            .query("SELECT ?fields FROM collision_alerts WHERE status = 'active' ORDER BY timestamp DESC")
            .fetch_all::<CollisionAlert>()
            .await?;
        Ok(rows)
    }

    pub async fn get_all_latest_telemetry(&self) -> Result<Vec<TelemetryData>> {
        let rows = self.client
            .query(
                "SELECT ?fields FROM telemetry \
                 WHERE (satellite_id, timestamp) IN (\
                   SELECT satellite_id, max(timestamp) \
                   FROM telemetry \
                   GROUP BY satellite_id\
                 ) \
                 ORDER BY satellite_id",
            )
            .fetch_all::<TelemetryData>()
            .await?;
        Ok(rows)
    }

    pub async fn get_tle_data(&self, satellite_id: u16) -> Result<Option<TleData>> {
        let rows = self.client
            .query("SELECT ?fields FROM tle_data WHERE satellite_id = ? ORDER BY timestamp DESC LIMIT 1")
            .bind(satellite_id)
            .fetch_all::<TleData>()
            .await?;
        Ok(rows.into_iter().next())
    }
}
