use chrono::{DateTime, Utc};
use clickhouse::Row;
use serde::{Deserialize, Serialize};
use uuid::Uuid;

#[derive(Debug, Clone, Serialize, Deserialize, Row)]
pub struct TelemetryData {
    pub satellite_id: u16,
    pub sequence_number: u64,
    pub timestamp: DateTime<Utc>,
    pub semi_major_axis: f64,
    pub eccentricity: f64,
    pub inclination: f64,
    pub raan: f64,
    pub arg_perigee: f64,
    pub true_anomaly: f64,
    pub quat_w: f64,
    pub quat_x: f64,
    pub quat_y: f64,
    pub quat_z: f64,
    pub propellant_remaining: f64,
    pub position_x: f64,
    pub position_y: f64,
    pub position_z: f64,
    pub velocity_x: f64,
    pub velocity_y: f64,
    pub velocity_z: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, Row)]
pub struct TleData {
    pub satellite_id: u16,
    pub timestamp: DateTime<Utc>,
    pub norad_id: String,
    pub line1: String,
    pub line2: String,
    pub epoch_year: f64,
    pub epoch_day: f64,
    pub mean_motion: f64,
    pub eccentricity_tle: f64,
    pub inclination_tle: f64,
    pub raan_tle: f64,
    pub arg_perigee_tle: f64,
    pub mean_anomaly_tle: f64,
    pub bstar: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, Row)]
pub struct CollisionAlert {
    pub alert_id: Uuid,
    pub timestamp: DateTime<Utc>,
    pub satellite_id_1: u16,
    pub satellite_id_2: u16,
    pub tca: DateTime<Utc>,
    pub miss_distance: f64,
    pub collision_probability: f64,
    pub alert_level: u8,
    pub status: String,
    pub maneuver_planned: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize, Row)]
pub struct OrbitManeuver {
    pub maneuver_id: Uuid,
    pub satellite_id: u16,
    pub timestamp: DateTime<Utc>,
    pub maneuver_type: String,
    pub delta_v_x: f64,
    pub delta_v_y: f64,
    pub delta_v_z: f64,
    pub fuel_cost: f64,
    pub target_semi_major_axis: f64,
    pub target_inclination: f64,
    pub executed: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize, Row)]
pub struct PropellantHistory {
    pub satellite_id: u16,
    pub timestamp: DateTime<Utc>,
    pub propellant_remaining: f64,
    pub consumption_rate: f64,
    pub estimated_lifetime_hours: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CollisionRiskLevel {
    Safe,
    Warning,
    Danger,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SatelliteStatusResponse {
    pub satellite_id: u16,
    pub name: String,
    pub current_position: Position3D,
    pub velocity: Velocity3D,
    pub orbital_elements: OrbitalElements,
    pub propellant: PropellantInfo,
    pub collision_risk_level: CollisionRiskLevel,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Position3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Velocity3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OrbitalElements {
    pub semi_major_axis: f64,
    pub eccentricity: f64,
    pub inclination: f64,
    pub raan: f64,
    pub arg_perigee: f64,
    pub true_anomaly: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PropellantInfo {
    pub remaining: f64,
    pub consumption_rate: f64,
    pub estimated_lifetime_hours: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CollisionAlertResponse {
    pub alert_id: Uuid,
    pub timestamp: DateTime<Utc>,
    pub satellite_id_1: u16,
    pub satellite_id_2: u16,
    pub satellite_name_1: String,
    pub satellite_name_2: String,
    pub tca: DateTime<Utc>,
    pub miss_distance: f64,
    pub collision_probability: f64,
    pub alert_level: u8,
    pub status: String,
    pub maneuver_planned: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OrbitManeuverResponse {
    pub maneuver_id: Uuid,
    pub satellite_id: u16,
    pub timestamp: DateTime<Utc>,
    pub maneuver_type: String,
    pub delta_v_x: f64,
    pub delta_v_y: f64,
    pub delta_v_z: f64,
    pub fuel_cost: f64,
    pub target_semi_major_axis: f64,
    pub target_inclination: f64,
    pub executed: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConstellationOverview {
    pub total_satellites: u32,
    pub active_alerts: u32,
    pub avg_propellant: f64,
    pub coverage_status: String,
}
