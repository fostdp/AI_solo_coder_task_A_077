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
}
