use std::collections::HashMap;
use std::time::Duration;
use tokio::sync::mpsc;
use crate::config::{AppConfig, Sgp4Config, NumericalPropagatorConfig as NumPropConfig, CollisionConfig};
use crate::models::{TleData, TelemetryData};

#[derive(Debug, Clone)]
pub struct Sgp4State {
    pub position_x: f64,
    pub position_y: f64,
    pub position_z: f64,
    pub velocity_x: f64,
    pub velocity_y: f64,
    pub velocity_z: f64,
    pub time_minutes: f64,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct TcaResult {
    pub tca_minutes: f64,
    pub miss_distance: f64,
    pub relative_velocity: f64,
    pub position1: (f64, f64, f64),
    pub position2: (f64, f64, f64),
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct CollisionAnalysis {
    pub satellite_id_1: u16,
    pub satellite_id_2: u16,
    pub tca_result: TcaResult,
    pub collision_probability: f64,
    pub alert_level: u8,
    pub encounter_point_eci: (f64, f64, f64),
}

#[derive(Debug, Clone)]
pub struct DualPropagatorResult {
    pub sgp4_state: Sgp4State,
    pub numerical_state: Sgp4State,
    pub position_divergence_km: f64,
    pub velocity_divergence_km_s: f64,
    pub corrected_state: Sgp4State,
}

pub struct Sgp4Propagator {
    mu_earth: f64,
    re_earth: f64,
    j2: f64,
    j3: f64,
    j4: f64,
    combined_radius: f64,
    kepler_max_iter: u32,
    kepler_tolerance: f64,
}

impl Sgp4Propagator {
    pub fn new(cfg: &Sgp4Config) -> Self {
        Self {
            mu_earth: cfg.mu_earth,
            re_earth: cfg.re_earth,
            j2: cfg.j2,
            j3: cfg.j3,
            j4: cfg.j4,
            combined_radius: cfg.combined_radius_km,
            kepler_max_iter: cfg.kepler_max_iterations,
            kepler_tolerance: cfg.kepler_tolerance,
        }
    }

    pub fn propagate(&self, tle: &TleData, minutes_from_epoch: f64) -> Sgp4State {
        let two_pi = 2.0 * std::f64::consts::PI;
        let n0 = tle.mean_motion * two_pi / 1440.0;
        let e0 = tle.eccentricity_tle;
        let i0 = tle.inclination_tle.to_radians();
        let omega0 = tle.arg_perigee_tle.to_radians();
        let raan0 = tle.raan_tle.to_radians();
        let m0 = tle.mean_anomaly_tle.to_radians();
        let bstar = tle.bstar;

        let a0 = (self.mu_earth / (n0 * n0)).powf(1.0 / 3.0);

        let cos_i0 = i0.cos();
        let sin_i0 = i0.sin();
        let e0sq = e0 * e0;
        let beta0sq = 1.0 - e0sq;
        let beta0 = beta0sq.sqrt();
        let theta2 = cos_i0 * cos_i0;
        let theta4 = theta2 * theta2;

        let a0e = a0 / self.re_earth;
        let a0e2 = a0e * a0e;

        let delta1 = 0.75 * self.j2 * (3.0 * theta2 - 1.0) / (a0e2 * beta0sq * beta0.sqrt());
        let a0_corr = a0 * (1.0 - delta1 / 3.0 - delta1 * delta1 / 9.0 - delta1 * delta1 * delta1 / 27.0);
        let a0e_corr = a0_corr / self.re_earth;
        let delta0 = 0.75 * self.j2 * (3.0 * theta2 - 1.0) / (a0e_corr * a0e_corr * beta0sq * beta0.sqrt());
        let n0_corr = n0 / (1.0 + delta0);
        let a0_final = (self.mu_earth / (n0_corr * n0_corr)).powf(1.0 / 3.0);
        let a0e_final = a0_final / self.re_earth;

        let raan_dot_j2 = -1.5 * self.j2 * n0_corr / (a0e_final * a0e_final * beta0sq * beta0sq) * cos_i0;
        let raan_dot_j4 = 0.3125 * self.j4 * n0_corr / (a0e_final.powi(4) * beta0sq.powi(5))
            * cos_i0 * (4.0 - 15.0 * theta2);
        let raan_dot = raan_dot_j2 + raan_dot_j4;

        let omega_dot_j2 = 0.75 * self.j2 * n0_corr
            / (a0e_final * a0e_final * beta0sq * beta0sq * beta0)
            * (5.0 * theta2 - 1.0);
        let omega_dot_j4 = -0.3125 * self.j4 * n0_corr / (a0e_final.powi(4) * beta0sq.powi(5))
            * (3.0 - 14.0 * theta2 + 18.0 * theta4);
        let omega_dot = omega_dot_j2 + omega_dot_j4;

        let delta_n = 0.75 * self.j2 * n0_corr
            / (a0e_final * a0e_final * beta0sq * beta0.sqrt())
            * (3.0 * theta2 - 1.0);

        let drag_rate = 1.5 * n0_corr * bstar / beta0sq;

        let t = minutes_from_epoch;

        let m_t = m0 + (n0_corr + delta_n) * t + drag_rate * t * t / 2.0;
        let omega_t = omega0 + omega_dot * t;
        let raan_t = raan0 + raan_dot * t;

        let j3_coeff = self.j3 / (2.0 * self.j2) * self.re_earth / (a0_final * beta0sq);

        let delta_e_lp = -j3_coeff * sin_i0 * omega_t.sin();

        let e_safe = e0.max(1e-8);
        let delta_omega_lp = j3_coeff * sin_i0 * cos_i0 / e_safe * omega_t.cos();

        let e_eff = (e0 + delta_e_lp).max(1e-8).min(0.9999);
        let omega_eff = omega_t + delta_omega_lp;

        let n_eff = n0_corr + delta_n + drag_rate * t;
        let a_eff = (self.mu_earth / (n_eff * n_eff)).powf(1.0 / 3.0);

        let e_anomaly = self.solve_kepler(m_t, e_eff);

        let true_anomaly =
            2.0 * (((1.0 + e_eff) / (1.0 - e_eff)).sqrt() * (e_anomaly / 2.0).tan()).atan();

        let e_eff_sq = e_eff * e_eff;
        let p = a_eff * (1.0 - e_eff_sq);
        let r = p / (1.0 + e_eff * true_anomaly.cos());

        let cos_nu = true_anomaly.cos();
        let sin_nu = true_anomaly.sin();

        let x_orb = r * cos_nu;
        let y_orb = r * sin_nu;

        let h = (self.mu_earth * p).sqrt();
        let vx_orb = -self.mu_earth / h * sin_nu;
        let vy_orb = self.mu_earth / h * (e_eff + cos_nu);

        let r_re = r / self.re_earth;
        let j2_sp = 0.75 * self.j2 / (r_re * r_re);
        let u = omega_eff + true_anomaly;
        let sin_u = u.sin();
        let sin2_u = (2.0 * u).sin();

        let delta_r_sp = self.re_earth * j2_sp * (1.0 - 3.0 * sin_i0 * sin_i0 * sin_u * sin_u);

        let delta_u_sp = -j2_sp / 2.0 * sin_i0 * sin_i0 * sin2_u;

        let delta_z_sp = -1.5 * self.j2 / (r_re * r_re) * sin_i0 * cos_i0 * sin_u;

        let u_corr = u + delta_u_sp;
        let r_corr = r + delta_r_sp;

        let x_orb_sp = r_corr * u_corr.cos();
        let y_orb_sp = r_corr * u_corr.sin();

        let cos_omega = omega_eff.cos();
        let sin_omega = omega_eff.sin();
        let cos_raan = raan_t.cos();
        let sin_raan = raan_t.sin();

        let (x, y, z) = Self::orbital_to_eci(
            x_orb_sp,
            y_orb_sp,
            cos_omega,
            sin_omega,
            cos_raan,
            sin_raan,
            cos_i0,
            sin_i0,
        );

        let z_corr = z + delta_z_sp * self.re_earth;

        let (vx, vy, vz) = Self::orbital_to_eci(
            vx_orb,
            vy_orb,
            cos_omega,
            sin_omega,
            cos_raan,
            sin_raan,
            cos_i0,
            sin_i0,
        );

        Sgp4State {
            position_x: x,
            position_y: y,
            position_z: z_corr,
            velocity_x: vx,
            velocity_y: vy,
            velocity_z: vz,
            time_minutes: minutes_from_epoch,
        }
    }

    #[inline]
    fn orbital_to_eci(
        x_orb: f64,
        y_orb: f64,
        cos_omega: f64,
        sin_omega: f64,
        cos_raan: f64,
        sin_raan: f64,
        cos_i: f64,
        sin_i: f64,
    ) -> (f64, f64, f64) {
        let x = (cos_omega * cos_raan - sin_omega * sin_raan * cos_i) * x_orb
            + (-sin_omega * cos_raan - cos_omega * sin_raan * cos_i) * y_orb;
        let y = (cos_omega * sin_raan + sin_omega * cos_raan * cos_i) * x_orb
            + (-sin_omega * sin_raan + cos_omega * cos_raan * cos_i) * y_orb;
        let z = sin_omega * sin_i * x_orb + cos_omega * sin_i * y_orb;
        (x, y, z)
    }

    fn solve_kepler(&self, m: f64, e: f64) -> f64 {
        let two_pi = 2.0 * std::f64::consts::PI;
        let mut m_norm = m % two_pi;
        if m_norm < 0.0 {
            m_norm += two_pi;
        }

        let mut e_anom = if e < 0.8 { m_norm } else { std::f64::consts::PI };

        for _ in 0..self.kepler_max_iter {
            let sin_e = e_anom.sin();
            let cos_e = e_anom.cos();
            let f = e_anom - e * sin_e - m_norm;
            let fp = 1.0 - e * cos_e;
            let delta = f / fp;
            e_anom -= delta;
            if delta.abs() < self.kepler_tolerance {
                break;
            }
        }

        e_anom
    }

    pub fn propagate_batch(
        &self,
        tle: &TleData,
        start_min: f64,
        end_min: f64,
        step_min: f64,
    ) -> Vec<Sgp4State> {
        if step_min <= 0.0 {
            return Vec::new();
        }
        let mut results = Vec::new();
        let mut t = start_min;
        while t <= end_min + step_min * 0.01 {
            results.push(self.propagate(tle, t));
            t += step_min;
        }
        results
    }
}

pub struct NumericalPropagator {
    mu_earth: f64,
    re_earth: f64,
    j2: f64,
    j3: f64,
    j4: f64,
    j5: f64,
    j6: f64,
    step_size_seconds: f64,
    include_j2: bool,
    include_j3: bool,
    include_j4: bool,
    include_j5_j6: bool,
    include_drag: bool,
    include_srp: bool,
    solar_activity_f107: f64,
    omega_earth: f64,
    srp_pressure: f64,
    reflectivity: f64,
    srp_area_mass: f64,
    drag_scale_height: f64,
    drag_rho0: f64,
    drag_h_ref: f64,
    drag_cd: f64,
    drag_area_mass: f64,
    divergence_threshold_km: f64,
}

impl NumericalPropagator {
    pub fn new(sgp4_cfg: &Sgp4Config, num_cfg: &NumPropConfig) -> Self {
        Self {
            mu_earth: sgp4_cfg.mu_earth,
            re_earth: sgp4_cfg.re_earth,
            j2: sgp4_cfg.j2,
            j3: sgp4_cfg.j3,
            j4: sgp4_cfg.j4,
            j5: sgp4_cfg.j5,
            j6: sgp4_cfg.j6,
            step_size_seconds: num_cfg.step_size_seconds,
            include_j2: num_cfg.include_j2,
            include_j3: num_cfg.include_j3,
            include_j4: num_cfg.include_j4,
            include_j5_j6: num_cfg.include_j5_j6,
            include_drag: num_cfg.include_drag,
            include_srp: num_cfg.include_srp,
            solar_activity_f107: num_cfg.solar_activity_f107,
            omega_earth: num_cfg.omega_earth,
            srp_pressure: num_cfg.srp_pressure,
            reflectivity: num_cfg.reflectivity,
            srp_area_mass: num_cfg.srp_area_mass,
            drag_scale_height: num_cfg.drag_scale_height,
            drag_rho0: num_cfg.drag_rho0,
            drag_h_ref: num_cfg.drag_h_ref,
            drag_cd: num_cfg.drag_cd,
            drag_area_mass: num_cfg.drag_area_mass,
            divergence_threshold_km: num_cfg.divergence_threshold_km,
        }
    }

    pub fn propagate_from_state(&self, state: &Sgp4State, duration_seconds: f64) -> Sgp4State {
        let mut s = [
            state.position_x,
            state.position_y,
            state.position_z,
            state.velocity_x,
            state.velocity_y,
            state.velocity_z,
        ];

        let dt = self.step_size_seconds;
        let mut remaining = duration_seconds;

        while remaining > 0.0 {
            let step = dt.min(remaining);
            s = self.rk4_step(&s, step);
            remaining -= step;
        }

        Sgp4State {
            position_x: s[0],
            position_y: s[1],
            position_z: s[2],
            velocity_x: s[3],
            velocity_y: s[4],
            velocity_z: s[5],
            time_minutes: state.time_minutes + duration_seconds / 60.0,
        }
    }

    pub fn propagate_from_tle(
        &self,
        tle: &TleData,
        sgp4: &Sgp4Propagator,
        minutes_from_epoch: f64,
    ) -> Sgp4State {
        let initial = sgp4.propagate(tle, 0.0);
        let duration_s = minutes_from_epoch * 60.0;
        self.propagate_from_state(&initial, duration_s)
    }

    pub fn propagate_batch_from_tle(
        &self,
        tle: &TleData,
        sgp4: &Sgp4Propagator,
        start_min: f64,
        end_min: f64,
        step_min: f64,
    ) -> Vec<Sgp4State> {
        if step_min <= 0.0 {
            return Vec::new();
        }
        let initial = sgp4.propagate(tle, 0.0);
        let mut results = Vec::new();
        let mut t = start_min;
        while t <= end_min + step_min * 0.01 {
            let state = self.propagate_from_state(&initial, t * 60.0);
            results.push(state);
            t += step_min;
        }
        results
    }

    pub fn compare_with_sgp4(
        &self,
        tle: &TleData,
        sgp4: &Sgp4Propagator,
        minutes_from_epoch: f64,
    ) -> DualPropagatorResult {
        let sgp4_state = sgp4.propagate(tle, minutes_from_epoch);
        let numerical_state = self.propagate_from_tle(tle, sgp4, minutes_from_epoch);

        let pos_div = ((sgp4_state.position_x - numerical_state.position_x).powi(2)
            + (sgp4_state.position_y - numerical_state.position_y).powi(2)
            + (sgp4_state.position_z - numerical_state.position_z).powi(2))
        .sqrt();

        let vel_div = ((sgp4_state.velocity_x - numerical_state.velocity_x).powi(2)
            + (sgp4_state.velocity_y - numerical_state.velocity_y).powi(2)
            + (sgp4_state.velocity_z - numerical_state.velocity_z).powi(2))
        .sqrt();

        let corrected_state = if pos_div > self.divergence_threshold_km {
            numerical_state.clone()
        } else {
            Sgp4State {
                position_x: (sgp4_state.position_x + numerical_state.position_x) / 2.0,
                position_y: (sgp4_state.position_y + numerical_state.position_y) / 2.0,
                position_z: (sgp4_state.position_z + numerical_state.position_z) / 2.0,
                velocity_x: (sgp4_state.velocity_x + numerical_state.velocity_x) / 2.0,
                velocity_y: (sgp4_state.velocity_y + numerical_state.velocity_y) / 2.0,
                velocity_z: (sgp4_state.velocity_z + numerical_state.velocity_z) / 2.0,
                time_minutes: minutes_from_epoch,
            }
        };

        DualPropagatorResult {
            sgp4_state,
            numerical_state,
            position_divergence_km: pos_div,
            velocity_divergence_km_s: vel_div,
            corrected_state,
        }
    }

    fn rk4_step(&self, state: &[f64; 6], dt: f64) -> [f64; 6] {
        let k1 = self.derivatives(state);

        let mut s2 = [0.0; 6];
        for i in 0..6 {
            s2[i] = state[i] + 0.5 * dt * k1[i];
        }
        let k2 = self.derivatives(&s2);

        let mut s3 = [0.0; 6];
        for i in 0..6 {
            s3[i] = state[i] + 0.5 * dt * k2[i];
        }
        let k3 = self.derivatives(&s3);

        let mut s4 = [0.0; 6];
        for i in 0..6 {
            s4[i] = state[i] + dt * k3[i];
        }
        let k4 = self.derivatives(&s4);

        let mut result = [0.0; 6];
        for i in 0..6 {
            result[i] = state[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
        result
    }

    fn derivatives(&self, state: &[f64; 6]) -> [f64; 6] {
        let pos = [state[0], state[1], state[2]];
        let vel = [state[3], state[4], state[5]];

        let acc = self.acceleration(&pos, &vel);

        [vel[0], vel[1], vel[2], acc[0], acc[1], acc[2]]
    }

    fn acceleration(&self, pos: &[f64; 3], vel: &[f64; 3]) -> [f64; 3] {
        let x = pos[0];
        let y = pos[1];
        let z = pos[2];
        let r = (x * x + y * y + z * z).sqrt();
        if r < 1.0 {
            return [0.0; 3];
        }

        let r2 = r * r;
        let r3 = r2 * r;
        let r5 = r3 * r2;
        let r7 = r5 * r2;

        let mut ax = -self.mu_earth * x / r3;
        let mut ay = -self.mu_earth * y / r3;
        let mut az = -self.mu_earth * z / r3;

        if self.include_j2 {
            let z2 = z * z;
            let fac = -1.5 * self.j2 * self.mu_earth * self.re_earth * self.re_earth / r5;
            let z2r2 = 5.0 * z2 / r2;
            ax += fac * x * (1.0 - z2r2);
            ay += fac * y * (1.0 - z2r2);
            az += fac * z * (3.0 - z2r2);
        }

        if self.include_j3 {
            let z3 = z * z * z;
            let fac = -2.5 * self.j3 * self.mu_earth * self.re_earth.powi(3) / r7;
            let z_r = z / r;
            ax += fac * x * (3.0 * z_r - 7.0 * z3 / r3);
            ay += fac * y * (3.0 * z_r - 7.0 * z3 / r3);
            az += fac * (6.0 * z2 / r2 - 7.0 * z2 * z2 / r2 / r2 - 1.5);
        }

        if self.include_j4 {
            let z2 = z * z;
            let z4 = z2 * z2;
            let fac = 1.875 * self.j4 * self.mu_earth * self.re_earth.powi(4) / r7;
            let z2r2 = z2 / r2;
            let common = 1.0 - 14.0 * z2r2 + 21.0 * z2r2 * z2r2;
            ax += fac * x * common;
            ay += fac * y * common;
            az += fac * z * (5.0 - 30.0 * z2r2 + 33.0 * z4 / r2 / r2);
        }

        if self.include_j5_j6 {
            let z2 = z * z;
            let z3 = z * z2;
            let fac5 = 2.1875 * self.j5 * self.mu_earth * self.re_earth.powi(5) / r7 / r2;
            let z_r = z / r;
            ax += fac5 * x * z_r * (5.0 - 21.0 * z2 / r2 + 33.0 * z2 * z2 / r2 / r2);
            ay += fac5 * y * z_r * (5.0 - 21.0 * z2 / r2 + 33.0 * z2 * z2 / r2 / r2);
            az += fac5 * (5.0 - 35.0 * z2 / r2 + 63.0 * z2 * z2 / r2 / r2) * z3 / z.max(1e-10);

            let fac6 = 1.5625 * self.j6 * self.mu_earth * self.re_earth.powi(6) / r7 / r2 / r2;
            let z2r2 = z2 / r2;
            let c6 = 1.0 - 27.0 * z2r2 + 99.0 * z2r2 * z2r2 - 429.0 / 35.0 * z2r2 * z2r2 * z2r2;
            ax += fac6 * x * c6;
            ay += fac6 * y * c6;
            az += fac6 * z * (7.0 - 63.0 * z2r2 + 99.0 * z2r2 * z2r2 + z2 * z2 * z2 / r2 / r2 / r2 * (-429.0 / 5.0));
        }

        if self.include_drag {
            let altitude = r - self.re_earth;
            if altitude > 0.0 && altitude < 2000.0 {
                let f107_factor = (0.01 * (self.solar_activity_f107 - 150.0)).exp();
                let rho = self.drag_rho0 * (-(altitude - self.drag_h_ref) / self.drag_scale_height).exp() * f107_factor;
                let v_rel_x = vel[0] + self.omega_earth * pos[1];
                let v_rel_y = vel[1] - self.omega_earth * pos[0];
                let v_rel_z = vel[2];
                let v_rel = (v_rel_x * v_rel_x + v_rel_y * v_rel_y + v_rel_z * v_rel_z).sqrt();
                if v_rel > 1e-10 {
                    let drag_fac = -0.5 * rho * v_rel * self.drag_cd * self.drag_area_mass;
                    ax += drag_fac * v_rel_x;
                    ay += drag_fac * v_rel_y;
                    az += drag_fac * v_rel_z;
                }
            }
        }

        if self.include_srp {
            let sun_x = 1.496e8;
            let sun_r = (sun_x * sun_x).sqrt();
            let srp_fac = self.srp_pressure * self.reflectivity * self.srp_area_mass / sun_r;
            let shadow = {
                let proj = (x * sun_x) / sun_r;
                if proj < 0.0 {
                    let perp2 = r2 - proj * proj;
                    perp2 > self.re_earth * self.re_earth
                } else {
                    true
                }
            };
            if shadow {
                ax += srp_fac * sun_x / sun_r;
            }
        }

        [ax, ay, az]
    }
}

pub struct CollisionProbabilityCalculator {
    coarse_scan_steps: u32,
    golden_section_iterations: u32,
    golden_section_tolerance: f64,
    alert_level1_probability: f64,
    alert_level2_probability: f64,
    sigma_along_track_m: f64,
    sigma_cross_track_m: f64,
    sigma_radial_m: f64,
    along_track_projection_weight: f64,
    combined_radius: f64,
    divergence_threshold_km: f64,
}

impl CollisionProbabilityCalculator {
    pub fn new(sgp4_cfg: &Sgp4Config, collision_cfg: &CollisionConfig, num_cfg: &NumPropConfig) -> Self {
        Self {
            coarse_scan_steps: collision_cfg.coarse_scan_steps,
            golden_section_iterations: collision_cfg.golden_section_iterations,
            golden_section_tolerance: collision_cfg.golden_section_tolerance,
            alert_level1_probability: collision_cfg.alert_level1_probability,
            alert_level2_probability: collision_cfg.alert_level2_probability,
            sigma_along_track_m: collision_cfg.sigma_along_track_m,
            sigma_cross_track_m: collision_cfg.sigma_cross_track_m,
            sigma_radial_m: collision_cfg.sigma_radial_m,
            along_track_projection_weight: collision_cfg.along_track_projection_weight,
            combined_radius: sgp4_cfg.combined_radius_km,
            divergence_threshold_km: num_cfg.divergence_threshold_km,
        }
    }

    pub fn compute_miss_distance(&self, state1: &Sgp4State, state2: &Sgp4State) -> f64 {
        let dx = state1.position_x - state2.position_x;
        let dy = state1.position_y - state2.position_y;
        let dz = state1.position_z - state2.position_z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    pub fn find_tca(
        &self,
        propagator: &Sgp4Propagator,
        tle1: &TleData,
        tle2: &TleData,
        search_start_min: f64,
        search_end_min: f64,
    ) -> TcaResult {
        let span = search_end_min - search_start_min;
        if span <= 0.0 {
            let s1 = propagator.propagate(tle1, search_start_min);
            let s2 = propagator.propagate(tle2, search_start_min);
            return self.build_tca_result(&s1, &s2, search_start_min);
        }

        let coarse_steps = self.coarse_scan_steps;
        let coarse_step = span / coarse_steps as f64;
        let mut best_t = search_start_min;
        let mut best_dist = f64::MAX;

        for k in 0..=coarse_steps {
            let t = search_start_min + k as f64 * coarse_step;
            let s1 = propagator.propagate(tle1, t);
            let s2 = propagator.propagate(tle2, t);
            let dist = self.compute_miss_distance(&s1, &s2);
            if dist < best_dist {
                best_dist = dist;
                best_t = t;
            }
        }

        let phi = (1.0 + 5_f64.sqrt()) / 2.0;
        let resphi = 2.0 - phi;
        let mut lo = (best_t - coarse_step).max(search_start_min);
        let mut hi = (best_t + coarse_step).min(search_end_min);

        let mut x1 = lo + resphi * (hi - lo);
        let mut x2 = hi - resphi * (hi - lo);

        let s1_a = propagator.propagate(tle1, x1);
        let s2_a = propagator.propagate(tle2, x1);
        let mut f1 = self.compute_miss_distance(&s1_a, &s2_a);

        let s1_b = propagator.propagate(tle1, x2);
        let s2_b = propagator.propagate(tle2, x2);
        let mut f2 = self.compute_miss_distance(&s1_b, &s2_b);

        for _ in 0..self.golden_section_iterations {
            if f1 < f2 {
                hi = x2;
                x2 = x1;
                f2 = f1;
                x1 = lo + resphi * (hi - lo);
                let s1 = propagator.propagate(tle1, x1);
                let s2 = propagator.propagate(tle2, x1);
                f1 = self.compute_miss_distance(&s1, &s2);
            } else {
                lo = x1;
                x1 = x2;
                f1 = f2;
                x2 = hi - resphi * (hi - lo);
                let s1 = propagator.propagate(tle1, x2);
                let s2 = propagator.propagate(tle2, x2);
                f2 = self.compute_miss_distance(&s1, &s2);
            }
            if (hi - lo).abs() < self.golden_section_tolerance {
                break;
            }
        }

        let tca_t = (lo + hi) / 2.0;
        let s1 = propagator.propagate(tle1, tca_t);
        let s2 = propagator.propagate(tle2, tca_t);
        self.build_tca_result(&s1, &s2, tca_t)
    }

    fn build_tca_result(&self, s1: &Sgp4State, s2: &Sgp4State, tca_min: f64) -> TcaResult {
        let miss = self.compute_miss_distance(s1, s2);
        let dvx = s1.velocity_x - s2.velocity_x;
        let dvy = s1.velocity_y - s2.velocity_y;
        let dvz = s1.velocity_z - s2.velocity_z;
        let rel_v = (dvx * dvx + dvy * dvy + dvz * dvz).sqrt();

        TcaResult {
            tca_minutes: tca_min,
            miss_distance: miss,
            relative_velocity: rel_v,
            position1: (s1.position_x, s1.position_y, s1.position_z),
            position2: (s2.position_x, s2.position_y, s2.position_z),
        }
    }

    pub fn compute_collision_probability(&self, tca: &TcaResult) -> f64 {
        let miss = tca.miss_distance;
        if miss < 1e-10 {
            return 1.0;
        }

        let dx = tca.position2.0 - tca.position1.0;
        let dy = tca.position2.1 - tca.position1.1;
        let dz = tca.position2.2 - tca.position1.2;

        let e_r = (dx / miss, dy / miss, dz / miss);

        let (ref_x, ref_y, ref_z) = if e_r.2.abs() < 0.9 {
            (1.0, 0.0, 0.0)
        } else {
            (0.0, 1.0, 0.0)
        };

        let ep1 = (
            ref_y * e_r.2 - ref_z * e_r.1,
            ref_z * e_r.0 - ref_x * e_r.2,
            ref_x * e_r.1 - ref_y * e_r.0,
        );
        let ep1_mag = (ep1.0 * ep1.0 + ep1.1 * ep1.1 + ep1.2 * ep1.2).sqrt();
        if ep1_mag < 1e-12 {
            return 0.0;
        }
        let ep1 = (ep1.0 / ep1_mag, ep1.1 / ep1_mag, ep1.2 / ep1_mag);

        let ep2 = (
            e_r.1 * ep1.2 - e_r.2 * ep1.1,
            e_r.2 * ep1.0 - e_r.0 * ep1.2,
            e_r.0 * ep1.1 - e_r.1 * ep1.0,
        );

        let sigma_at = self.sigma_along_track_m / 1000.0;
        let sigma_ct = self.sigma_cross_track_m / 1000.0;
        let sigma_rad = self.sigma_radial_m / 1000.0;

        let sig_at_comb = sigma_at * std::f64::consts::SQRT_2;
        let sig_ct_comb = sigma_ct * std::f64::consts::SQRT_2;
        let sig_rad_comb = sigma_rad * std::f64::consts::SQRT_2;

        let at_weight = self.along_track_projection_weight;
        let sig_b1_sq = sig_ct_comb * sig_ct_comb + at_weight * sig_at_comb * sig_at_comb;
        let sig_b2_sq = sig_rad_comb * sig_rad_comb + at_weight * sig_at_comb * sig_at_comb;
        let sig_b1 = sig_b1_sq.sqrt();
        let sig_b2 = sig_b2_sq.sqrt();

        let x_b = dx * ep1.0 + dy * ep1.1 + dz * ep1.2;
        let y_b = dx * ep2.0 + dy * ep2.1 + dz * ep2.2;

        let mahalanobis_sq = x_b * x_b / sig_b1_sq + y_b * y_b / sig_b2_sq;

        (self.combined_radius * self.combined_radius) / (2.0 * sig_b1 * sig_b2)
            * (-0.5 * mahalanobis_sq).exp()
    }

    pub fn analyze_pair(
        &self,
        propagator: &Sgp4Propagator,
        tle1: &TleData,
        tle2: &TleData,
        horizon_hours: f64,
    ) -> CollisionAnalysis {
        let end_min = horizon_hours * 60.0;
        let tca = self.find_tca(propagator, tle1, tle2, 0.0, end_min);
        let collision_prob = self.compute_collision_probability(&tca);

        let alert_level = if collision_prob > self.alert_level2_probability {
            2
        } else if collision_prob > self.alert_level1_probability {
            1
        } else {
            0
        };

        let encounter = (
            (tca.position1.0 + tca.position2.0) / 2.0,
            (tca.position1.1 + tca.position2.1) / 2.0,
            (tca.position1.2 + tca.position2.2) / 2.0,
        );

        CollisionAnalysis {
            satellite_id_1: tle1.satellite_id,
            satellite_id_2: tle2.satellite_id,
            tca_result: tca,
            collision_probability: collision_prob,
            alert_level,
            encounter_point_eci: encounter,
        }
    }

    pub fn analyze_pair_dual(
        &self,
        sgp4: &Sgp4Propagator,
        numerical: &NumericalPropagator,
        tle1: &TleData,
        tle2: &TleData,
        horizon_hours: f64,
    ) -> CollisionAnalysis {
        let end_min = horizon_hours * 60.0;

        let sgp4_s1 = sgp4.propagate(tle1, end_min);
        let sgp4_s2 = sgp4.propagate(tle2, end_min);
        let num_s1 = numerical.propagate_from_tle(tle1, sgp4, end_min);
        let num_s2 = numerical.propagate_from_tle(tle2, sgp4, end_min);

        let div1 = ((sgp4_s1.position_x - num_s1.position_x).powi(2)
            + (sgp4_s1.position_y - num_s1.position_y).powi(2)
            + (sgp4_s1.position_z - num_s1.position_z).powi(2))
        .sqrt();
        let div2 = ((sgp4_s2.position_x - num_s2.position_x).powi(2)
            + (sgp4_s2.position_y - num_s2.position_y).powi(2)
            + (sgp4_s2.position_z - num_s2.position_z).powi(2))
        .sqrt();

        let use_numerical = div1 > self.divergence_threshold_km || div2 > self.divergence_threshold_km;
        if use_numerical {
            tracing::info!(
                "High SGP4/numerical divergence: sat{}={:.3}km, sat{}={:.3}km — using numerical propagator for TCA",
                tle1.satellite_id, div1, tle2.satellite_id, div2
            );
        }

        let tca = if use_numerical {
            self.find_tca_with_propagator(tle1, tle2, 0.0, end_min, |tle, t_min| {
                numerical.propagate_from_tle(tle, sgp4, t_min)
            })
        } else {
            self.find_tca(sgp4, tle1, tle2, 0.0, end_min)
        };

        let collision_prob = self.compute_collision_probability(&tca);

        let alert_level = if collision_prob > self.alert_level2_probability {
            2
        } else if collision_prob > self.alert_level1_probability {
            1
        } else {
            0
        };

        let encounter = (
            (tca.position1.0 + tca.position2.0) / 2.0,
            (tca.position1.1 + tca.position2.1) / 2.0,
            (tca.position1.2 + tca.position2.2) / 2.0,
        );

        CollisionAnalysis {
            satellite_id_1: tle1.satellite_id,
            satellite_id_2: tle2.satellite_id,
            tca_result: tca,
            collision_probability: collision_prob,
            alert_level,
            encounter_point_eci: encounter,
        }
    }

    fn find_tca_with_propagator<F>(
        &self,
        tle1: &TleData,
        tle2: &TleData,
        search_start_min: f64,
        search_end_min: f64,
        propagate: F,
    ) -> TcaResult
    where
        F: Fn(&TleData, f64) -> Sgp4State,
    {
        let span = search_end_min - search_start_min;
        if span <= 0.0 {
            let s1 = propagate(tle1, search_start_min);
            let s2 = propagate(tle2, search_start_min);
            return self.build_tca_result(&s1, &s2, search_start_min);
        }

        let coarse_steps = self.coarse_scan_steps;
        let coarse_step = span / coarse_steps as f64;
        let mut best_t = search_start_min;
        let mut best_dist = f64::MAX;

        for k in 0..=coarse_steps {
            let t = search_start_min + k as f64 * coarse_step;
            let s1 = propagate(tle1, t);
            let s2 = propagate(tle2, t);
            let dist = self.compute_miss_distance(&s1, &s2);
            if dist < best_dist {
                best_dist = dist;
                best_t = t;
            }
        }

        let phi = (1.0 + 5_f64.sqrt()) / 2.0;
        let resphi = 2.0 - phi;
        let mut lo = (best_t - coarse_step).max(search_start_min);
        let mut hi = (best_t + coarse_step).min(search_end_min);

        let mut x1 = lo + resphi * (hi - lo);
        let mut x2 = hi - resphi * (hi - lo);

        let s1_a = propagate(tle1, x1);
        let s2_a = propagate(tle2, x1);
        let mut f1 = self.compute_miss_distance(&s1_a, &s2_a);

        let s1_b = propagate(tle1, x2);
        let s2_b = propagate(tle2, x2);
        let mut f2 = self.compute_miss_distance(&s1_b, &s2_b);

        for _ in 0..self.golden_section_iterations {
            if f1 < f2 {
                hi = x2;
                x2 = x1;
                f2 = f1;
                x1 = lo + resphi * (hi - lo);
                let s1 = propagate(tle1, x1);
                let s2 = propagate(tle2, x1);
                f1 = self.compute_miss_distance(&s1, &s2);
            } else {
                lo = x1;
                x1 = x2;
                f1 = f2;
                x2 = hi - resphi * (hi - lo);
                let s1 = propagate(tle1, x2);
                let s2 = propagate(tle2, x2);
                f2 = self.compute_miss_distance(&s1, &s2);
            }
            if (hi - lo).abs() < self.golden_section_tolerance {
                break;
            }
        }

        let tca_t = (lo + hi) / 2.0;
        let s1 = propagate(tle1, tca_t);
        let s2 = propagate(tle2, tca_t);
        self.build_tca_result(&s1, &s2, tca_t)
    }
}

pub struct CollisionPredictor {
    propagator: Sgp4Propagator,
    numerical_propagator: NumericalPropagator,
    calculator: CollisionProbabilityCalculator,
    config: CollisionConfig,
    latest_telemetry: HashMap<u16, TelemetryData>,
    tle_cache: HashMap<u16, TleData>,
    active_analyses: Vec<CollisionAnalysis>,
}

impl CollisionPredictor {
    pub fn new(app_config: &AppConfig) -> Self {
        Self {
            propagator: Sgp4Propagator::new(&app_config.sgp4),
            numerical_propagator: NumericalPropagator::new(&app_config.sgp4, &app_config.numerical_propagator),
            calculator: CollisionProbabilityCalculator::new(&app_config.sgp4, &app_config.collision, &app_config.numerical_propagator),
            config: app_config.collision.clone(),
            latest_telemetry: HashMap::new(),
            tle_cache: HashMap::new(),
            active_analyses: Vec::new(),
        }
    }

    pub async fn run(
        mut self,
        mut telemetry_rx: mpsc::Receiver<TelemetryData>,
        mut tle_rx: mpsc::Receiver<TleData>,
        analysis_tx: mpsc::Sender<CollisionAnalysis>,
    ) {
        let mut interval = tokio::time::interval(
            Duration::from_secs(self.config.analysis_interval_seconds)
        );
        interval.tick().await;

        let mut telemetry_done = false;
        let mut tle_done = false;

        loop {
            if telemetry_done && tle_done {
                break;
            }

            tokio::select! {
                telemetry = telemetry_rx.recv(), if !telemetry_done => {
                    match telemetry {
                        Some(t) => {
                            self.latest_telemetry.insert(t.satellite_id, t);
                        }
                        None => {
                            telemetry_done = true;
                        }
                    }
                }
                tle = tle_rx.recv(), if !tle_done => {
                    match tle {
                        Some(t) => {
                            self.tle_cache.insert(t.satellite_id, t);
                        }
                        None => {
                            tle_done = true;
                        }
                    }
                }
                _ = interval.tick() => {
                    if self.tle_cache.len() < 2 {
                        continue;
                    }

                    let ids: Vec<u16> = self.tle_cache.keys().copied().collect();
                    let mut new_analyses = Vec::new();

                    for i in 0..ids.len() {
                        for j in (i + 1)..ids.len() {
                            let tle1 = &self.tle_cache[&ids[i]];
                            let tle2 = &self.tle_cache[&ids[j]];

                            let analysis = self.calculator.analyze_pair_dual(
                                &self.propagator,
                                &self.numerical_propagator,
                                tle1,
                                tle2,
                                self.config.horizon_hours,
                            );

                            new_analyses.push(analysis.clone());

                            if analysis_tx.send(analysis).await.is_err() {
                                return;
                            }
                        }
                    }

                    self.active_analyses = new_analyses;
                }
            }
        }
    }
}
