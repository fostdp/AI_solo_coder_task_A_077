use crate::models::TleData;

const MU_EARTH: f64 = 398600.4418;
const RE_EARTH: f64 = 6378.137;
const J2: f64 = 1.08263e-3;
const J3: f64 = -2.532e-6;
const J4: f64 = -1.6109e-6;
const COMBINED_RADIUS: f64 = 0.01;
const TWO_PI: f64 = 2.0 * std::f64::consts::PI;

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

#[derive(Debug, Clone)]
pub struct TcaResult {
    pub tca_minutes: f64,
    pub miss_distance: f64,
    pub relative_velocity: f64,
    pub position1: (f64, f64, f64),
    pub position2: (f64, f64, f64),
}

#[derive(Debug, Clone)]
pub struct CollisionAnalysis {
    pub satellite_id_1: u16,
    pub satellite_id_2: u16,
    pub tca_result: TcaResult,
    pub collision_probability: f64,
    pub alert_level: u8,
    pub encounter_point_eci: (f64, f64, f64),
}

pub struct Sgp4Propagator;

impl Sgp4Propagator {
    pub fn new() -> Self {
        Self
    }

    pub fn propagate(&self, tle: &TleData, minutes_from_epoch: f64) -> Sgp4State {
        let n0 = tle.mean_motion * TWO_PI / 1440.0;
        let e0 = tle.eccentricity_tle;
        let i0 = tle.inclination_tle.to_radians();
        let omega0 = tle.arg_perigee_tle.to_radians();
        let raan0 = tle.raan_tle.to_radians();
        let m0 = tle.mean_anomaly_tle.to_radians();
        let bstar = tle.bstar;

        let a0 = (MU_EARTH / (n0 * n0)).powf(1.0 / 3.0);

        let cos_i0 = i0.cos();
        let sin_i0 = i0.sin();
        let e0sq = e0 * e0;
        let beta0sq = 1.0 - e0sq;
        let beta0 = beta0sq.sqrt();
        let theta2 = cos_i0 * cos_i0;
        let theta4 = theta2 * theta2;

        let a0e = a0 / RE_EARTH;
        let a0e2 = a0e * a0e;

        // SGP4 initialization: recover corrected mean motion and semi-major axis
        // using first-order J2 correction (Vallado SGP4 init procedure)
        let delta1 = 0.75 * J2 * (3.0 * theta2 - 1.0) / (a0e2 * beta0sq * beta0.sqrt());
        let a0_corr = a0 * (1.0 - delta1 / 3.0 - delta1 * delta1 / 9.0 - delta1 * delta1 * delta1 / 27.0);
        let a0e_corr = a0_corr / RE_EARTH;
        let delta0 = 0.75 * J2 * (3.0 * theta2 - 1.0) / (a0e_corr * a0e_corr * beta0sq * beta0.sqrt());
        let n0_corr = n0 / (1.0 + delta0);
        let a0_final = (MU_EARTH / (n0_corr * n0_corr)).powf(1.0 / 3.0);
        let a0e_final = a0_final / RE_EARTH;

        // ============ Secular perturbations ============

        // RAAN rate (rad/min): dominant J2 + smaller J4
        let raan_dot_j2 = -1.5 * J2 * n0_corr / (a0e_final * a0e_final * beta0sq * beta0sq) * cos_i0;
        let raan_dot_j4 = 0.3125 * J4 * n0_corr / (a0e_final.powi(4) * beta0sq.powi(5))
            * cos_i0 * (4.0 - 15.0 * theta2);
        let raan_dot = raan_dot_j2 + raan_dot_j4;

        // Argument of perigee rate (rad/min): J2 + J4
        let omega_dot_j2 = 0.75 * J2 * n0_corr
            / (a0e_final * a0e_final * beta0sq * beta0sq * beta0)
            * (5.0 * theta2 - 1.0);
        let omega_dot_j4 = -0.3125 * J4 * n0_corr / (a0e_final.powi(4) * beta0sq.powi(5))
            * (3.0 - 14.0 * theta2 + 18.0 * theta4);
        let omega_dot = omega_dot_j2 + omega_dot_j4;

        // Mean motion secular correction from J2 (rad/min)
        let delta_n = 0.75 * J2 * n0_corr
            / (a0e_final * a0e_final * beta0sq * beta0.sqrt())
            * (3.0 * theta2 - 1.0);

        // ============ Atmospheric drag (B*) ============
        // B* drives secular decay: positive B* means orbit decays,
        // semi-major axis shrinks, mean motion increases.
        // drag_rate has units rad/min^2.
        let drag_rate = 1.5 * n0_corr * bstar / beta0sq;

        // ============ Propagate to time t ============
        let t = minutes_from_epoch;

        // Mean anomaly: M(t) = M0 + (n0_corr + delta_n)*t + drag_rate*t^2/2
        let m_t = m0 + (n0_corr + delta_n) * t + drag_rate * t * t / 2.0;
        let omega_t = omega0 + omega_dot * t;
        let raan_t = raan0 + raan_dot * t;

        // ============ Long-period perturbations (J3) ============
        // J3 produces long-period oscillations in e and omega.
        let j3_coeff = J3 / (2.0 * J2) * RE_EARTH / (a0_final * beta0sq);

        let delta_e_lp = -j3_coeff * sin_i0 * omega_t.sin();

        let e_safe = e0.max(1e-8);
        let delta_omega_lp = j3_coeff * sin_i0 * cos_i0 / e_safe * omega_t.cos();

        let e_eff = (e0 + delta_e_lp).max(1e-8).min(0.9999);
        let omega_eff = omega_t + delta_omega_lp;

        // Effective mean motion with accumulated drag
        let n_eff = n0_corr + delta_n + drag_rate * t;
        let a_eff = (MU_EARTH / (n_eff * n_eff)).powf(1.0 / 3.0);

        // ============ Solve Kepler's equation: M = E - e*sin(E) ============
        let e_anomaly = Self::solve_kepler(m_t, e_eff);

        // ============ True anomaly from eccentric anomaly ============
        // tan(nu/2) = sqrt((1+e)/(1-e)) * tan(E/2)
        let true_anomaly =
            2.0 * (((1.0 + e_eff) / (1.0 - e_eff)).sqrt() * (e_anomaly / 2.0).tan()).atan();

        // ============ Position and velocity in orbital plane ============
        let e_eff_sq = e_eff * e_eff;
        let p = a_eff * (1.0 - e_eff_sq);
        let r = p / (1.0 + e_eff * true_anomaly.cos());

        let cos_nu = true_anomaly.cos();
        let sin_nu = true_anomaly.sin();

        let x_orb = r * cos_nu;
        let y_orb = r * sin_nu;

        let h = (MU_EARTH * p).sqrt();
        let vx_orb = -MU_EARTH / h * sin_nu;
        let vy_orb = MU_EARTH / h * (e_eff + cos_nu);

        // ============ Short-period J2 perturbations ============
        let r_re = r / RE_EARTH;
        let j2_sp = 0.75 * J2 / (r_re * r_re);
        let u = omega_eff + true_anomaly;
        let sin_u = u.sin();
        let sin2_u = (2.0 * u).sin();

        // Radial short-period correction
        let delta_r_sp = RE_EARTH * j2_sp * (1.0 - 3.0 * sin_i0 * sin_i0 * sin_u * sin_u);

        // In-track (argument of latitude) short-period correction
        let delta_u_sp = -j2_sp / 2.0 * sin_i0 * sin_i0 * sin2_u;

        // Out-of-plane short-period correction
        let delta_z_sp = -1.5 * J2 / (r_re * r_re) * sin_i0 * cos_i0 * sin_u;

        // Corrected orbital-plane position with short-period terms
        let u_corr = u + delta_u_sp;
        let r_corr = r + delta_r_sp;

        let x_orb_sp = r_corr * u_corr.cos();
        let y_orb_sp = r_corr * u_corr.sin();

        // ============ Rotate to ECI frame ============
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

        // Add out-of-plane short-period correction
        let z_corr = z + delta_z_sp * RE_EARTH;

        // Velocity rotation (without short-period corrections;
        // short-period velocity terms are small and high-frequency)
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

    fn solve_kepler(m: f64, e: f64) -> f64 {
        let mut m_norm = m % TWO_PI;
        if m_norm < 0.0 {
            m_norm += TWO_PI;
        }

        let mut e_anom = if e < 0.8 { m_norm } else { std::f64::consts::PI };

        for _ in 0..50 {
            let sin_e = e_anom.sin();
            let cos_e = e_anom.cos();
            let f = e_anom - e * sin_e - m_norm;
            let fp = 1.0 - e * cos_e;
            let delta = f / fp;
            e_anom -= delta;
            if delta.abs() < 1e-14 {
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

pub struct CollisionProbabilityCalculator;

impl CollisionProbabilityCalculator {
    pub fn new() -> Self {
        Self
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

        // Phase 1: Coarse scan with 200 steps to bracket the minimum
        let coarse_steps = 200;
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

        // Phase 2: Golden-section search refinement
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

        for _ in 0..60 {
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
            if (hi - lo).abs() < 1e-10 {
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

        // ============ B-plane construction ============
        // At TCA, the relative position is approximately in the B-plane
        // (perpendicular to V_rel). We construct a frame using the
        // relative position geometry.

        let e_r = (dx / miss, dy / miss, dz / miss);

        // Choose reference vector perpendicular to e_r
        let (ref_x, ref_y, ref_z) = if e_r.2.abs() < 0.9 {
            (1.0, 0.0, 0.0)
        } else {
            (0.0, 1.0, 0.0)
        };

        // e_perp1 = ref × e_r  (cross-track-like)
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

        // e_perp2 = e_r × e_perp1
        let ep2 = (
            e_r.1 * ep1.2 - e_r.2 * ep1.1,
            e_r.2 * ep1.0 - e_r.0 * ep1.2,
            e_r.0 * ep1.1 - e_r.1 * ep1.0,
        );

        // ============ Position uncertainties ============
        // Per-satellite 1σ (km): along-track=100m, cross-track=10m, radial=10m
        let sigma_at = 0.1;
        let sigma_ct = 0.01;
        let sigma_rad = 0.01;

        // Combined uncertainties for two independent objects (RSS)
        let sig_at_comb = sigma_at * std::f64::consts::SQRT_2;
        let sig_ct_comb = sigma_ct * std::f64::consts::SQRT_2;
        let sig_rad_comb = sigma_rad * std::f64::consts::SQRT_2;

        // B-plane uncertainties: cross-track and radial project strongly,
        // along-track projects weakly (mostly along V_rel which is
        // perpendicular to the B-plane at TCA).
        let sig_b1_sq = sig_ct_comb * sig_ct_comb + 0.1 * sig_at_comb * sig_at_comb;
        let sig_b2_sq = sig_rad_comb * sig_rad_comb + 0.1 * sig_at_comb * sig_at_comb;
        let sig_b1 = sig_b1_sq.sqrt();
        let sig_b2 = sig_b2_sq.sqrt();

        // ============ Miss vector in B-plane ============
        let x_b = dx * ep1.0 + dy * ep1.1 + dz * ep1.2;
        let y_b = dx * ep2.0 + dy * ep2.1 + dz * ep2.2;

        // ============ Chan (2008) 2D Gaussian probability ============
        // Small hard-body radius approximation:
        //   P_c ≈ (R_c² / (2 σ₁ σ₂)) × exp(-½ (x²/σ₁² + y²/σ₂²))
        let mahalanobis_sq = x_b * x_b / sig_b1_sq + y_b * y_b / sig_b2_sq;

        (COMBINED_RADIUS * COMBINED_RADIUS) / (2.0 * sig_b1 * sig_b2)
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

        let alert_level = if collision_prob > 1e-3 {
            2
        } else if collision_prob > 1e-4 {
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

        let use_numerical = div1 > 1.0 || div2 > 1.0;
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

        let alert_level = if collision_prob > 1e-3 {
            2
        } else if collision_prob > 1e-4 {
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

        let coarse_steps = 200;
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

        for _ in 0..60 {
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
            if (hi - lo).abs() < 1e-10 {
                break;
            }
        }

        let tca_t = (lo + hi) / 2.0;
        let s1 = propagate(tle1, tca_t);
        let s2 = propagate(tle2, tca_t);
        self.build_tca_result(&s1, &s2, tca_t)
    }
}

const J5: f64 = -2.7e-7;
const J6: f64 = 3.4e-7;
const OMEGA_EARTH: f64 = 7.2921159e-5;
const SRP_PRESSURE: f64 = 4.56e-6;
const REFLECTIVITY: f64 = 1.5;
const SRP_AREA_MASS: f64 = 0.01;
const DRAG_SCALE_HEIGHT: f64 = 58.515;
const DRAG_RHO_0: f64 = 6.967e-13;
const DRAG_H_REF: f64 = 500.0;
const DRAG_CD: f64 = 2.2;
const DRAG_AREA_MASS: f64 = 0.01;

#[derive(Debug, Clone)]
pub struct NumericalPropagatorConfig {
    pub step_size_seconds: f64,
    pub include_j2: bool,
    pub include_j3: bool,
    pub include_j4: bool,
    pub include_j5_j6: bool,
    pub include_drag: bool,
    pub include_srp: bool,
    pub solar_activity_f107: f64,
}

impl Default for NumericalPropagatorConfig {
    fn default() -> Self {
        Self {
            step_size_seconds: 30.0,
            include_j2: true,
            include_j3: true,
            include_j4: true,
            include_j5_j6: true,
            include_drag: true,
            include_srp: true,
            solar_activity_f107: 150.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct DualPropagatorResult {
    pub sgp4_state: Sgp4State,
    pub numerical_state: Sgp4State,
    pub position_divergence_km: f64,
    pub velocity_divergence_km_s: f64,
    pub corrected_state: Sgp4State,
}

pub struct NumericalPropagator {
    config: NumericalPropagatorConfig,
}

impl NumericalPropagator {
    pub fn new(config: NumericalPropagatorConfig) -> Self {
        Self { config }
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

        let dt = self.config.step_size_seconds;
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

        let corrected_state = if pos_div > 1.0 {
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

        let mut ax = -MU_EARTH * x / r3;
        let mut ay = -MU_EARTH * y / r3;
        let mut az = -MU_EARTH * z / r3;

        if self.config.include_j2 {
            let z2 = z * z;
            let fac = -1.5 * J2 * MU_EARTH * RE_EARTH * RE_EARTH / r5;
            let z2r2 = 5.0 * z2 / r2;
            ax += fac * x * (1.0 - z2r2);
            ay += fac * y * (1.0 - z2r2);
            az += fac * z * (3.0 - z2r2);
        }

        if self.config.include_j3 {
            let z3 = z * z * z;
            let fac = -2.5 * J3 * MU_EARTH * RE_EARTH.powi(3) / r7;
            let z_r = z / r;
            ax += fac * x * (3.0 * z_r - 7.0 * z3 / r3);
            ay += fac * y * (3.0 * z_r - 7.0 * z3 / r3);
            az += fac * (6.0 * z2 / r2 - 7.0 * z2 * z2 / r2 / r2 - 1.5);
        }

        if self.config.include_j4 {
            let z2 = z * z;
            let z4 = z2 * z2;
            let fac = 1.875 * J4 * MU_EARTH * RE_EARTH.powi(4) / r7;
            let z2r2 = z2 / r2;
            let common = 1.0 - 14.0 * z2r2 + 21.0 * z2r2 * z2r2;
            ax += fac * x * common;
            ay += fac * y * common;
            az += fac * z * (5.0 - 30.0 * z2r2 + 33.0 * z4 / r2 / r2);
        }

        if self.config.include_j5_j6 {
            let z2 = z * z;
            let z3 = z * z2;
            let fac5 = 2.1875 * J5 * MU_EARTH * RE_EARTH.powi(5) / r7 / r2;
            let z_r = z / r;
            ax += fac5 * x * z_r * (5.0 - 21.0 * z2 / r2 + 33.0 * z2 * z2 / r2 / r2);
            ay += fac5 * y * z_r * (5.0 - 21.0 * z2 / r2 + 33.0 * z2 * z2 / r2 / r2);
            az += fac5 * (5.0 - 35.0 * z2 / r2 + 63.0 * z2 * z2 / r2 / r2) * z3 / z.max(1e-10);

            let fac6 = 1.5625 * J6 * MU_EARTH * RE_EARTH.powi(6) / r7 / r2 / r2;
            let z2r2 = z2 / r2;
            let c6 = 1.0 - 27.0 * z2r2 + 99.0 * z2r2 * z2r2 - 429.0 / 35.0 * z2r2 * z2r2 * z2r2;
            ax += fac6 * x * c6;
            ay += fac6 * y * c6;
            az += fac6 * z * (7.0 - 63.0 * z2r2 + 99.0 * z2r2 * z2r2 + z2 * z2 * z2 / r2 / r2 / r2 * (-429.0 / 5.0));
        }

        if self.config.include_drag {
            let altitude = r - RE_EARTH;
            if altitude > 0.0 && altitude < 2000.0 {
                let f107_factor = (0.01 * (self.config.solar_activity_f107 - 150.0)).exp();
                let rho = DRAG_RHO_0 * (-(altitude - DRAG_H_REF) / DRAG_SCALE_HEIGHT).exp() * f107_factor;
                let v_rel_x = vel[0] + OMEGA_EARTH * pos[1];
                let v_rel_y = vel[1] - OMEGA_EARTH * pos[0];
                let v_rel_z = vel[2];
                let v_rel = (v_rel_x * v_rel_x + v_rel_y * v_rel_y + v_rel_z * v_rel_z).sqrt();
                if v_rel > 1e-10 {
                    let drag_fac = -0.5 * rho * v_rel * DRAG_CD * DRAG_AREA_MASS;
                    ax += drag_fac * v_rel_x;
                    ay += drag_fac * v_rel_y;
                    az += drag_fac * v_rel_z;
                }
            }
        }

        if self.config.include_srp {
            let sun_x = 1.496e8;
            let sun_r = (sun_x * sun_x).sqrt();
            let srp_fac = SRP_PRESSURE * REFLECTIVITY * SRP_AREA_MASS / sun_r;
            let shadow = {
                let proj = (x * sun_x) / sun_r;
                if proj < 0.0 {
                    let perp2 = r2 - proj * proj;
                    perp2 > RE_EARTH * RE_EARTH
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
