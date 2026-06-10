use crate::models::{CollisionAlert, OrbitManeuver, TelemetryData, TleData};
use crate::sgp4_engine::{CollisionAnalysis, CollisionProbabilityCalculator, Sgp4Propagator};
use chrono::Utc;
use rand::Rng;
use uuid::Uuid;

const SCALE_HEIGHT: f64 = 58.515;
const RHO_0: f64 = 6.967e-13;
const H_REF: f64 = 500.0;
const CD: f64 = 2.2;
const AREA_MASS_RATIO: f64 = 0.01;
const RE_EARTH: f64 = 6378.137;
const ISP: f64 = 220.0;
const G0: f64 = 9.80665e-3;
const DRY_MASS: f64 = 200.0;

pub struct AtmosphericDragModel;

impl AtmosphericDragModel {
    pub fn new() -> Self {
        Self
    }

    pub fn atmospheric_density(&self, altitude_km: f64) -> f64 {
        if altitude_km < 0.0 {
            return RHO_0;
        }
        RHO_0 * (-(altitude_km - H_REF) / SCALE_HEIGHT).exp()
    }

    pub fn drag_deceleration(&self, altitude_km: f64, velocity_km_s: f64) -> f64 {
        let rho = self.atmospheric_density(altitude_km);
        0.5 * rho * velocity_km_s * velocity_km_s * CD * AREA_MASS_RATIO
    }

    pub fn orbit_decay_rate(&self, semi_major_axis_km: f64, eccentricity: f64) -> f64 {
        let altitude = semi_major_axis_km - RE_EARTH;
        let rho = self.atmospheric_density(altitude);
        let v_approx = (398600.4418 / semi_major_axis_km).sqrt();
        let a_drag = 0.5 * rho * v_approx * v_approx * CD * AREA_MASS_RATIO;
        let period_s = 2.0 * std::f64::consts::PI * (semi_major_axis_km.powi(3) / 398600.4418).sqrt();
        let n_orbits_per_day = 86400.0 / period_s;
        let beta = (1.0 - eccentricity * eccentricity).sqrt();
        -2.0 * semi_major_axis_km * semi_major_axis_km * a_drag / (398600.4418 * beta) * n_orbits_per_day
    }
}

#[derive(Debug, Clone)]
pub struct ManeuverPlan {
    pub satellite_id: u16,
    pub delta_v_x: f64,
    pub delta_v_y: f64,
    pub delta_v_z: f64,
    pub fuel_cost: f64,
    pub target_semi_major_axis: f64,
    pub estimated_orbit_lifetime_days: f64,
    pub fitness_score: f64,
}

impl ManeuverPlan {
    fn compute_fuel_cost(dv_total: f64) -> f64 {
        if dv_total <= 0.0 {
            return 0.0;
        }
        DRY_MASS * (1.0 - (1.0 - dv_total / (ISP * G0)).exp())
    }
}

#[derive(Debug, Clone)]
struct Individual {
    dv_radial: f64,
    dv_along: f64,
    dv_cross: f64,
    fitness: f64,
}

pub struct GeneticOrbitOptimizer {
    population_size: usize,
    generations: usize,
    mutation_rate: f64,
}

impl GeneticOrbitOptimizer {
    pub fn new(population_size: usize, generations: usize, mutation_rate: f64) -> Self {
        Self {
            population_size,
            generations,
            mutation_rate,
        }
    }

    pub fn optimize_station_keeping(
        &self,
        telemetry: &TelemetryData,
        target_sma: f64,
    ) -> ManeuverPlan {
        let mut rng = rand::thread_rng();
        let mut population: Vec<Individual> = (0..self.population_size)
            .map(|_| Self::random_individual_station_keeping(&mut rng))
            .collect();

        for ind in population.iter_mut() {
            ind.fitness = self.station_keeping_fitness(ind, telemetry, target_sma);
        }

        for _ in 0..self.generations {
            let mut new_pop = Vec::with_capacity(self.population_size);

            new_pop.push(self.best_individual(&population).clone());

            while new_pop.len() < self.population_size {
                let p1 = self.tournament_select(&population, &mut rng);
                let p2 = self.tournament_select(&population, &mut rng);
                let mut child = self.blx_alpha_crossover(&p1, &p2, 0.5, &mut rng);
                self.mutate(&mut child, &mut rng);
                child.fitness = self.station_keeping_fitness(&child, telemetry, target_sma);
                new_pop.push(child);
            }

            population = new_pop;
        }

        let best = self.best_individual(&population);
        let dv_total = (best.dv_radial * best.dv_radial
            + best.dv_along * best.dv_along
            + best.dv_cross * best.dv_cross)
        .sqrt();
        let fuel = ManeuverPlan::compute_fuel_cost(dv_total);

        let drag_model = AtmosphericDragModel::new();
        let new_sma = telemetry.semi_major_axis + best.dv_along * 100.0;
        let decay_rate = drag_model.orbit_decay_rate(new_sma, telemetry.eccentricity);
        let lifetime = if decay_rate.abs() > 1e-10 {
            (telemetry.propellant_remaining - fuel).max(0.0) / (decay_rate.abs() * 0.1).max(1e-10)
        } else {
            3650.0
        };

        ManeuverPlan {
            satellite_id: telemetry.satellite_id,
            delta_v_x: best.dv_radial,
            delta_v_y: best.dv_along,
            delta_v_z: best.dv_cross,
            fuel_cost: fuel,
            target_semi_major_axis: target_sma,
            estimated_orbit_lifetime_days: lifetime.min(3650.0),
            fitness_score: best.fitness,
        }
    }

    pub fn optimize_avoidance_maneuver(
        &self,
        telemetry: &TelemetryData,
        tle1: &TleData,
        tle2: &TleData,
        propagator: &Sgp4Propagator,
    ) -> ManeuverPlan {
        let mut rng = rand::thread_rng();
        let calculator = CollisionProbabilityCalculator::new();
        let baseline = calculator.analyze_pair(propagator, tle1, tle2, 72.0);

        let mut population: Vec<Individual> = (0..self.population_size)
            .map(|_| Self::random_individual_avoidance(&mut rng))
            .collect();

        for ind in population.iter_mut() {
            ind.fitness = self.avoidance_fitness(ind, telemetry, tle1, tle2, propagator, &calculator);
        }

        for _ in 0..self.generations {
            let mut new_pop = Vec::with_capacity(self.population_size);
            new_pop.push(self.best_individual(&population).clone());

            while new_pop.len() < self.population_size {
                let p1 = self.tournament_select(&population, &mut rng);
                let p2 = self.tournament_select(&population, &mut rng);
                let mut child = self.blx_alpha_crossover(&p1, &p2, 0.5, &mut rng);
                self.mutate_avoidance(&mut child, &mut rng);
                child.fitness = self.avoidance_fitness(&child, telemetry, tle1, tle2, propagator, &calculator);
                new_pop.push(child);
            }

            population = new_pop;
        }

        let best = self.best_individual(&population);
        let dv_total = (best.dv_radial * best.dv_radial
            + best.dv_along * best.dv_along
            + best.dv_cross * best.dv_cross)
        .sqrt();
        let fuel = ManeuverPlan::compute_fuel_cost(dv_total);

        ManeuverPlan {
            satellite_id: telemetry.satellite_id,
            delta_v_x: best.dv_radial,
            delta_v_y: best.dv_along,
            delta_v_z: best.dv_cross,
            fuel_cost: fuel,
            target_semi_major_axis: telemetry.semi_major_axis,
            estimated_orbit_lifetime_days: 0.0,
            fitness_score: best.fitness,
        }
    }

    fn station_keeping_fitness(&self, ind: &Individual, telemetry: &TelemetryData, target_sma: f64) -> f64 {
        let dv_total = (ind.dv_radial * ind.dv_radial
            + ind.dv_along * ind.dv_along
            + ind.dv_cross * ind.dv_cross)
        .sqrt();

        let fuel = ManeuverPlan::compute_fuel_cost(dv_total);
        if fuel > telemetry.propellant_remaining {
            return -1e6;
        }

        let delta_sma = telemetry.semi_major_axis + ind.dv_along * 100.0 - target_sma;
        let sma_penalty = if delta_sma.abs() > 2.0 {
            delta_sma * delta_sma * 100.0
        } else {
            0.0
        };

        let new_ecc = telemetry.eccentricity + (ind.dv_radial * 0.001).abs();
        let ecc_penalty = if new_ecc > 0.002 {
            (new_ecc - 0.001) * 1000.0
        } else {
            0.0
        };

        let fuel_penalty = fuel * 10.0;

        let drag_model = AtmosphericDragModel::new();
        let new_sma = telemetry.semi_major_axis + ind.dv_along * 100.0;
        let decay_rate = drag_model.orbit_decay_rate(new_sma, new_ecc);
        let remaining_fuel = telemetry.propellant_remaining - fuel;
        let lifetime_bonus = if decay_rate.abs() > 1e-10 {
            (remaining_fuel / (decay_rate.abs() * 0.1).max(1e-10)).min(3650.0)
        } else {
            3650.0
        };

        -(sma_penalty + ecc_penalty + fuel_penalty) + lifetime_bonus * 0.01
    }

    fn avoidance_fitness(
        &self,
        ind: &Individual,
        telemetry: &TelemetryData,
        tle1: &TleData,
        tle2: &TleData,
        propagator: &Sgp4Propagator,
        calculator: &CollisionProbabilityCalculator,
    ) -> f64 {
        let dv_total = (ind.dv_radial * ind.dv_radial
            + ind.dv_along * ind.dv_along
            + ind.dv_cross * ind.dv_cross)
        .sqrt();
        let fuel = ManeuverPlan::compute_fuel_cost(dv_total);

        if fuel > telemetry.propellant_remaining {
            return -1e8;
        }

        let mut tle_mod = tle1.clone();
        let dv_along_orbits = ind.dv_along / (tle_mod.mean_motion * 2.0 * std::f64::consts::PI / 1440.0).sqrt();
        tle_mod.mean_motion += ind.dv_along * 0.001;

        let analysis = calculator.analyze_pair(propagator, &tle_mod, tle2, 72.0);

        -analysis.collision_probability * 1e6 + 0.01 * fuel
    }

    fn random_individual_station_keeping(rng: &mut rand::rngs::ThreadRng) -> Individual {
        Individual {
            dv_radial: rng.gen_range(-0.5..0.5),
            dv_along: rng.gen_range(-2.0..2.0),
            dv_cross: rng.gen_range(-0.3..0.3),
            fitness: 0.0,
        }
    }

    fn random_individual_avoidance(rng: &mut rand::rngs::ThreadRng) -> Individual {
        Individual {
            dv_radial: rng.gen_range(-1.0..1.0),
            dv_along: rng.gen_range(-5.0..5.0),
            dv_cross: rng.gen_range(-2.0..2.0),
            fitness: 0.0,
        }
    }

    fn tournament_select<'a>(
        &self,
        pop: &'a [Individual],
        rng: &mut rand::rngs::ThreadRng,
    ) -> &'a Individual {
        let mut best = &pop[rng.gen_range(0..pop.len())];
        for _ in 0..2 {
            let challenger = &pop[rng.gen_range(0..pop.len())];
            if challenger.fitness > best.fitness {
                best = challenger;
            }
        }
        best
    }

    fn blx_alpha_crossover(
        &self,
        p1: &Individual,
        p2: &Individual,
        alpha: f64,
        rng: &mut rand::rngs::ThreadRng,
    ) -> Individual {
        let dv_r = Self::blx_blend(p1.dv_radial, p2.dv_radial, alpha, rng);
        let dv_a = Self::blx_blend(p1.dv_along, p2.dv_along, alpha, rng);
        let dv_c = Self::blx_blend(p1.dv_cross, p2.dv_cross, alpha, rng);
        Individual {
            dv_radial: dv_r,
            dv_along: dv_a,
            dv_cross: dv_c,
            fitness: 0.0,
        }
    }

    fn blx_blend(a: f64, b: f64, alpha: f64, rng: &mut rand::rngs::ThreadRng) -> f64 {
        let min_val = a.min(b);
        let max_val = a.max(b);
        let range = max_val - min_val;
        rng.gen_range((min_val - alpha * range)..(max_val + alpha * range))
    }

    fn mutate(&self, ind: &mut Individual, rng: &mut rand::rngs::ThreadRng) {
        if rng.gen::<f64>() < self.mutation_rate {
            ind.dv_radial += rng.gen_range(-0.2..0.2);
        }
        if rng.gen::<f64>() < self.mutation_rate {
            ind.dv_along += rng.gen_range(-0.5..0.5);
        }
        if rng.gen::<f64>() < self.mutation_rate {
            ind.dv_cross += rng.gen_range(-0.1..0.1);
        }
    }

    fn mutate_avoidance(&self, ind: &mut Individual, rng: &mut rand::rngs::ThreadRng) {
        if rng.gen::<f64>() < self.mutation_rate {
            ind.dv_radial += rng.gen_range(-0.5..0.5);
        }
        if rng.gen::<f64>() < self.mutation_rate {
            ind.dv_along += rng.gen_range(-1.0..1.0);
        }
        if rng.gen::<f64>() < self.mutation_rate {
            ind.dv_cross += rng.gen_range(-0.5..0.5);
        }
    }

    fn best_individual<'a>(&self, pop: &'a [Individual]) -> &'a Individual {
        pop.iter().max_by(|a, b| a.fitness.partial_cmp(&b.fitness).unwrap()).unwrap()
    }
}

pub struct AlertManager;

impl AlertManager {
    pub fn new() -> Self {
        Self
    }

    pub fn evaluate_collision(&self, analysis: &CollisionAnalysis) -> Option<CollisionAlert> {
        if analysis.alert_level == 0 {
            return None;
        }

        let tca_time = Utc::now() + chrono::Duration::minutes(analysis.tca_result.tca_minutes as i64);

        Some(CollisionAlert {
            alert_id: Uuid::new_v4(),
            timestamp: Utc::now(),
            satellite_id_1: analysis.satellite_id_1,
            satellite_id_2: analysis.satellite_id_2,
            tca: tca_time,
            miss_distance: analysis.tca_result.miss_distance,
            collision_probability: analysis.collision_probability,
            alert_level: analysis.alert_level,
            status: "active".to_string(),
            maneuver_planned: analysis.alert_level == 2,
        })
    }

    pub fn compute_emergency_avoidance(
        &self,
        analysis: &CollisionAnalysis,
        optimizer: &GeneticOrbitOptimizer,
        telemetry1: &TelemetryData,
        _telemetry2: &TelemetryData,
        tle1: &TleData,
        tle2: &TleData,
        propagator: &Sgp4Propagator,
    ) -> (CollisionAlert, OrbitManeuver) {
        let alert = self.evaluate_collision(analysis).unwrap_or_else(|| CollisionAlert {
            alert_id: Uuid::new_v4(),
            timestamp: Utc::now(),
            satellite_id_1: analysis.satellite_id_1,
            satellite_id_2: analysis.satellite_id_2,
            tca: Utc::now() + chrono::Duration::minutes(analysis.tca_result.tca_minutes as i64),
            miss_distance: analysis.tca_result.miss_distance,
            collision_probability: analysis.collision_probability,
            alert_level: 2,
            status: "active".to_string(),
            maneuver_planned: true,
        });

        let plan = optimizer.optimize_avoidance_maneuver(telemetry1, tle1, tle2, propagator);

        let maneuver = OrbitManeuver {
            maneuver_id: Uuid::new_v4(),
            satellite_id: telemetry1.satellite_id,
            timestamp: Utc::now(),
            maneuver_type: "collision_avoidance".to_string(),
            delta_v_x: plan.delta_v_x,
            delta_v_y: plan.delta_v_y,
            delta_v_z: plan.delta_v_z,
            fuel_cost: plan.fuel_cost,
            target_semi_major_axis: telemetry1.semi_major_axis,
            target_inclination: telemetry1.inclination,
            executed: false,
        };

        (alert, maneuver)
    }

    pub async fn push_alert_to_ground_station(&self, alert: &CollisionAlert) -> anyhow::Result<()> {
        let client = reqwest::Client::new();
        let _resp = client
            .post("http://localhost:8888/ground-station/alert")
            .json(alert)
            .timeout(std::time::Duration::from_secs(5))
            .send()
            .await?;
        tracing::info!(
            "Alert pushed to ground station: sat1={}, sat2={}, level={}",
            alert.satellite_id_1,
            alert.satellite_id_2,
            alert.alert_level
        );
        Ok(())
    }

    pub async fn push_maneuver_to_ground_station(&self, maneuver: &OrbitManeuver) -> anyhow::Result<()> {
        let client = reqwest::Client::new();
        let _resp = client
            .post("http://localhost:8888/ground-station/maneuver")
            .json(maneuver)
            .timeout(std::time::Duration::from_secs(5))
            .send()
            .await?;
        tracing::info!(
            "Maneuver pushed to ground station: sat={}, type={}",
            maneuver.satellite_id,
            maneuver.maneuver_type
        );
        Ok(())
    }
}
