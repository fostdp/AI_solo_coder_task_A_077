use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct AppConfig {
    pub sgp4: Sgp4Config,
    pub numerical_propagator: NumericalPropagatorConfig,
    pub collision: CollisionConfig,
    pub optimizer: OptimizerConfig,
    pub atmosphere: AtmosphereConfig,
    pub ground_station: GroundStationConfig,
    pub network: NetworkConfig,
    pub reorder_buffer: ReorderBufferConfig,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Sgp4Config {
    pub mu_earth: f64,
    pub re_earth: f64,
    pub j2: f64,
    pub j3: f64,
    pub j4: f64,
    pub j5: f64,
    pub j6: f64,
    pub combined_radius_km: f64,
    pub kepler_max_iterations: u32,
    pub kepler_tolerance: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct NumericalPropagatorConfig {
    pub step_size_seconds: f64,
    pub include_j2: bool,
    pub include_j3: bool,
    pub include_j4: bool,
    pub include_j5_j6: bool,
    pub include_drag: bool,
    pub include_srp: bool,
    pub solar_activity_f107: f64,
    pub omega_earth: f64,
    pub srp_pressure: f64,
    pub reflectivity: f64,
    pub srp_area_mass: f64,
    pub drag_scale_height: f64,
    pub drag_rho0: f64,
    pub drag_h_ref: f64,
    pub drag_cd: f64,
    pub drag_area_mass: f64,
    pub divergence_threshold_km: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct CollisionConfig {
    pub coarse_scan_steps: u32,
    pub golden_section_iterations: u32,
    pub golden_section_tolerance: f64,
    pub horizon_hours: f64,
    pub analysis_interval_seconds: u64,
    pub alert_level1_probability: f64,
    pub alert_level2_probability: f64,
    pub sigma_along_track_m: f64,
    pub sigma_cross_track_m: f64,
    pub sigma_radial_m: f64,
    pub along_track_projection_weight: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct OptimizerConfig {
    pub population_size: usize,
    pub generations: usize,
    pub mutation_rate: f64,
    pub num_islands: usize,
    pub migration_interval: usize,
    pub migration_count: usize,
    pub blx_alpha: f64,
    pub tournament_k: usize,
    pub isp_seconds: f64,
    pub g0_km_s2: f64,
    pub dry_mass_kg: f64,
    pub dv_radial_range_station: [f64; 2],
    pub dv_along_range_station: [f64; 2],
    pub dv_cross_range_station: [f64; 2],
    pub dv_radial_range_avoidance: [f64; 2],
    pub dv_along_range_avoidance: [f64; 2],
    pub dv_cross_range_avoidance: [f64; 2],
}

#[derive(Debug, Clone, Deserialize)]
pub struct AtmosphereConfig {
    pub scale_height: f64,
    pub rho0: f64,
    pub h_ref: f64,
    pub cd: f64,
    pub area_mass_ratio: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct GroundStationConfig {
    pub alert_url: String,
    pub maneuver_url: String,
    pub push_timeout_seconds: u64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct NetworkConfig {
    pub telemetry_udp_port: u16,
    pub tle_udp_port: u16,
    pub http_port: u16,
    pub clickhouse_url: String,
    pub clickhouse_database: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ReorderBufferConfig {
    pub max_buffer_size: usize,
}

impl AppConfig {
    pub fn load_from_file(path: &str) -> anyhow::Result<Self> {
        let content = std::fs::read_to_string(path)?;
        let config: AppConfig = toml::from_str(&content)?;
        Ok(config)
    }

    pub fn load() -> anyhow::Result<Self> {
        let default_path = "config.toml";
        let env_path = std::env::var("CONFIG_PATH").unwrap_or_else(|_| default_path.to_string());
        Self::load_from_file(&env_path)
    }
}
