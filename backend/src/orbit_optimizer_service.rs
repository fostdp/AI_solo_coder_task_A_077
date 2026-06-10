use crate::collision_predictor::{CollisionAnalysis, CollisionProbabilityCalculator, Sgp4Propagator};
use crate::config::{AppConfig, AtmosphereConfig, GroundStationConfig, OptimizerConfig};
use crate::models::{CollisionAlert, OrbitManeuver, TelemetryData, TleData};
use chrono::Utc;
use rand::Rng;
use std::collections::HashMap;
use tokio::sync::{mpsc, oneshot};
use uuid::Uuid;

const RE_EARTH: f64 = 6378.137;
const MU_EARTH: f64 = 398600.4418;

pub struct AtmosphericDragModel {
    scale_height: f64,
    rho0: f64,
    h_ref: f64,
    cd: f64,
    area_mass_ratio: f64,
}

impl AtmosphericDragModel {
    pub fn new(config: &AtmosphereConfig) -> Self {
        Self {
            scale_height: config.scale_height,
            rho0: config.rho0,
            h_ref: config.h_ref,
            cd: config.cd,
            area_mass_ratio: config.area_mass_ratio,
        }
    }

    pub fn atmospheric_density(&self, altitude_km: f64) -> f64 {
        if altitude_km < 0.0 {
            return self.rho0;
        }
        self.rho0 * (-(altitude_km - self.h_ref) / self.scale_height).exp()
    }

    pub fn drag_deceleration(&self, altitude_km: f64, velocity_km_s: f64) -> f64 {
        let rho = self.atmospheric_density(altitude_km);
        0.5 * rho * velocity_km_s * velocity_km_s * self.cd * self.area_mass_ratio
    }

    pub fn orbit_decay_rate(&self, semi_major_axis_km: f64, eccentricity: f64) -> f64 {
        let altitude = semi_major_axis_km - RE_EARTH;
        let rho = self.atmospheric_density(altitude);
        let v_approx = (MU_EARTH / semi_major_axis_km).sqrt();
        let a_drag = 0.5 * rho * v_approx * v_approx * self.cd * self.area_mass_ratio;
        let period_s = 2.0 * std::f64::consts::PI * (semi_major_axis_km.powi(3) / MU_EARTH).sqrt();
        let n_orbits_per_day = 86400.0 / period_s;
        let beta = (1.0 - eccentricity * eccentricity).sqrt();
        -2.0 * semi_major_axis_km * semi_major_axis_km * a_drag / (MU_EARTH * beta) * n_orbits_per_day
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
    fn compute_fuel_cost(dv_total: f64, isp: f64, g0: f64, dry_mass: f64) -> f64 {
        if dv_total <= 0.0 {
            return 0.0;
        }
        dry_mass * (1.0 - (1.0 - dv_total / (isp * g0)).exp())
    }
}

#[derive(Debug, Clone)]
struct Individual {
    dv_radial: f64,
    dv_along: f64,
    dv_cross: f64,
    fitness: f64,
}

struct Island {
    population: Vec<Individual>,
    best_fitness: f64,
}

pub struct CoEvolutionOrbitOptimizer {
    config: OptimizerConfig,
    drag_model: AtmosphericDragModel,
}

impl CoEvolutionOrbitOptimizer {
    pub fn new(optimizer_config: OptimizerConfig, atmosphere_config: AtmosphereConfig) -> Self {
        let drag_model = AtmosphericDragModel::new(&atmosphere_config);
        Self {
            config: optimizer_config,
            drag_model,
        }
    }

    pub fn optimize_station_keeping(
        &self,
        telemetry: &TelemetryData,
        target_sma: f64,
    ) -> ManeuverPlan {
        let mut rng = rand::thread_rng();
        let island_pop_size = (self.config.population_size / self.config.num_islands).max(1);

        let mut islands: Vec<Island> = (0..self.config.num_islands)
            .map(|_| {
                let pop: Vec<Individual> = (0..island_pop_size)
                    .map(|_| self.random_individual_station_keeping(&mut rng))
                    .collect();
                let mut island = Island {
                    population: pop,
                    best_fitness: f64::NEG_INFINITY,
                };
                for ind in island.population.iter_mut() {
                    ind.fitness = self.station_keeping_fitness(ind, telemetry, target_sma);
                }
                island.best_fitness = self.best_individual(&island.population).fitness;
                island
            })
            .collect();

        for gen in 0..self.config.generations {
            for island in islands.iter_mut() {
                let mut new_pop = Vec::with_capacity(island_pop_size);
                new_pop.push(self.best_individual(&island.population).clone());
                while new_pop.len() < island_pop_size {
                    let p1 = self.tournament_select(&island.population, &mut rng);
                    let p2 = self.tournament_select(&island.population, &mut rng);
                    let mut child = self.blx_alpha_crossover(&p1, &p2, &mut rng);
                    self.mutate(&mut child, &mut rng);
                    child.fitness = self.station_keeping_fitness(&child, telemetry, target_sma);
                    new_pop.push(child);
                }
                island.population = new_pop;
                island.best_fitness = self.best_individual(&island.population).fitness;
            }

            if (gen + 1) % self.config.migration_interval == 0 && islands.len() > 1 {
                self.migrate(&mut islands);
            }
        }

        let best = islands
            .iter()
            .flat_map(|island| island.population.iter())
            .max_by(|a, b| a.fitness.partial_cmp(&b.fitness).unwrap())
            .unwrap();

        self.build_station_keeping_plan(best, telemetry, target_sma)
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
        let _baseline = calculator.analyze_pair(propagator, tle1, tle2, 72.0);

        let island_pop_size = (self.config.population_size / self.config.num_islands).max(1);

        let mut islands: Vec<Island> = (0..self.config.num_islands)
            .map(|_| {
                let pop: Vec<Individual> = (0..island_pop_size)
                    .map(|_| self.random_individual_avoidance(&mut rng))
                    .collect();
                let mut island = Island {
                    population: pop,
                    best_fitness: f64::NEG_INFINITY,
                };
                for ind in island.population.iter_mut() {
                    ind.fitness =
                        self.avoidance_fitness(ind, telemetry, tle1, tle2, propagator, &calculator);
                }
                island.best_fitness = self.best_individual(&island.population).fitness;
                island
            })
            .collect();

        for gen in 0..self.config.generations {
            for island in islands.iter_mut() {
                let mut new_pop = Vec::with_capacity(island_pop_size);
                new_pop.push(self.best_individual(&island.population).clone());
                while new_pop.len() < island_pop_size {
                    let p1 = self.tournament_select(&island.population, &mut rng);
                    let p2 = self.tournament_select(&island.population, &mut rng);
                    let mut child = self.blx_alpha_crossover(&p1, &p2, &mut rng);
                    self.mutate_avoidance(&mut child, &mut rng);
                    child.fitness = self.avoidance_fitness(
                        &child,
                        telemetry,
                        tle1,
                        tle2,
                        propagator,
                        &calculator,
                    );
                    new_pop.push(child);
                }
                island.population = new_pop;
                island.best_fitness = self.best_individual(&island.population).fitness;
            }

            if (gen + 1) % self.config.migration_interval == 0 && islands.len() > 1 {
                self.migrate(&mut islands);
            }
        }

        let best = islands
            .iter()
            .flat_map(|island| island.population.iter())
            .max_by(|a, b| a.fitness.partial_cmp(&b.fitness).unwrap())
            .unwrap();

        let dv_total = (best.dv_radial * best.dv_radial
            + best.dv_along * best.dv_along
            + best.dv_cross * best.dv_cross)
        .sqrt();
        let fuel = ManeuverPlan::compute_fuel_cost(
            dv_total,
            self.config.isp_seconds,
            self.config.g0_km_s2,
            self.config.dry_mass_kg,
        );

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

    pub fn optimize_constellation_station_keeping(
        &self,
        telemetry_map: &HashMap<u16, TelemetryData>,
        target_sma: f64,
    ) -> Vec<ManeuverPlan> {
        let mut planes: HashMap<u16, Vec<&TelemetryData>> = HashMap::new();
        for telemetry in telemetry_map.values() {
            let plane = telemetry.satellite_id / 16;
            planes.entry(plane).or_default().push(telemetry);
        }

        let mut plans = Vec::new();

        for mut sats in planes.into_values() {
            sats.sort_by_key(|t| t.satellite_id);

            let mut neighbor_results: Vec<(f64, Individual)> = Vec::new();
            let mut plane_plans: Vec<ManeuverPlan> = Vec::new();

            for telemetry in &sats {
                let mut rng = rand::thread_rng();
                let island_pop_size = (self.config.population_size / self.config.num_islands).max(1);

                let mut islands: Vec<Island> = (0..self.config.num_islands)
                    .map(|_| {
                        let pop: Vec<Individual> = (0..island_pop_size)
                            .map(|_| self.random_individual_station_keeping(&mut rng))
                            .collect();
                        let mut island = Island {
                            population: pop,
                            best_fitness: f64::NEG_INFINITY,
                        };
                        for ind in island.population.iter_mut() {
                            ind.fitness = self.constellation_fitness(
                                ind,
                                telemetry,
                                target_sma,
                                &neighbor_results,
                            );
                        }
                        island.best_fitness = self.best_individual(&island.population).fitness;
                        island
                    })
                    .collect();

                for gen in 0..self.config.generations {
                    for island in islands.iter_mut() {
                        let mut new_pop = Vec::with_capacity(island_pop_size);
                        new_pop.push(self.best_individual(&island.population).clone());
                        while new_pop.len() < island_pop_size {
                            let p1 = self.tournament_select(&island.population, &mut rng);
                            let p2 = self.tournament_select(&island.population, &mut rng);
                            let mut child = self.blx_alpha_crossover(&p1, &p2, &mut rng);
                            self.mutate(&mut child, &mut rng);
                            child.fitness = self.constellation_fitness(
                                &child,
                                telemetry,
                                target_sma,
                                &neighbor_results,
                            );
                            new_pop.push(child);
                        }
                        island.population = new_pop;
                        island.best_fitness = self.best_individual(&island.population).fitness;
                    }

                    if (gen + 1) % self.config.migration_interval == 0 && islands.len() > 1 {
                        self.migrate(&mut islands);
                    }
                }

                let best = islands
                    .iter()
                    .flat_map(|island| island.population.iter())
                    .max_by(|a, b| a.fitness.partial_cmp(&b.fitness).unwrap())
                    .unwrap();

                neighbor_results.push((telemetry.semi_major_axis, best.clone()));

                let plan = self.build_station_keeping_plan(best, telemetry, target_sma);
                plane_plans.push(plan);
            }

            plans.extend(plane_plans);
        }

        plans
    }

    fn build_station_keeping_plan(
        &self,
        best: &Individual,
        telemetry: &TelemetryData,
        target_sma: f64,
    ) -> ManeuverPlan {
        let dv_total = (best.dv_radial * best.dv_radial
            + best.dv_along * best.dv_along
            + best.dv_cross * best.dv_cross)
        .sqrt();
        let fuel = ManeuverPlan::compute_fuel_cost(
            dv_total,
            self.config.isp_seconds,
            self.config.g0_km_s2,
            self.config.dry_mass_kg,
        );

        let new_sma = telemetry.semi_major_axis + best.dv_along * 100.0;
        let decay_rate = self.drag_model.orbit_decay_rate(new_sma, telemetry.eccentricity);
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

    fn constellation_fitness(
        &self,
        ind: &Individual,
        telemetry: &TelemetryData,
        target_sma: f64,
        neighbor_results: &[(f64, Individual)],
    ) -> f64 {
        let mut fitness = self.station_keeping_fitness(ind, telemetry, target_sma);
        let post_sma = telemetry.semi_major_axis + ind.dv_along * 100.0;
        for (neighbor_sma, neighbor_ind) in neighbor_results {
            let neighbor_post_sma = *neighbor_sma + neighbor_ind.dv_along * 100.0;
            let distance = (post_sma - neighbor_post_sma).abs();
            if distance < 10.0 {
                fitness -= (10.0 - distance) * 1000.0;
            }
        }
        fitness
    }

    fn station_keeping_fitness(&self, ind: &Individual, telemetry: &TelemetryData, target_sma: f64) -> f64 {
        let dv_total = (ind.dv_radial * ind.dv_radial
            + ind.dv_along * ind.dv_along
            + ind.dv_cross * ind.dv_cross)
        .sqrt();

        let fuel = ManeuverPlan::compute_fuel_cost(
            dv_total,
            self.config.isp_seconds,
            self.config.g0_km_s2,
            self.config.dry_mass_kg,
        );

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

        let new_sma = telemetry.semi_major_axis + ind.dv_along * 100.0;
        let decay_rate = self.drag_model.orbit_decay_rate(new_sma, new_ecc);
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
        let fuel = ManeuverPlan::compute_fuel_cost(
            dv_total,
            self.config.isp_seconds,
            self.config.g0_km_s2,
            self.config.dry_mass_kg,
        );

        if fuel > telemetry.propellant_remaining {
            return -1e8;
        }

        let mut tle_mod = tle1.clone();
        let _dv_along_orbits = ind.dv_along / (tle_mod.mean_motion * 2.0 * std::f64::consts::PI / 1440.0).sqrt();
        tle_mod.mean_motion += ind.dv_along * 0.001;

        let analysis = calculator.analyze_pair(propagator, &tle_mod, tle2, 72.0);

        -analysis.collision_probability * 1e6 + 0.01 * fuel
    }

    fn random_individual_station_keeping(&self, rng: &mut rand::rngs::ThreadRng) -> Individual {
        Individual {
            dv_radial: rng.gen_range(self.config.dv_radial_range_station[0]..self.config.dv_radial_range_station[1]),
            dv_along: rng.gen_range(self.config.dv_along_range_station[0]..self.config.dv_along_range_station[1]),
            dv_cross: rng.gen_range(self.config.dv_cross_range_station[0]..self.config.dv_cross_range_station[1]),
            fitness: 0.0,
        }
    }

    fn random_individual_avoidance(&self, rng: &mut rand::rngs::ThreadRng) -> Individual {
        Individual {
            dv_radial: rng.gen_range(self.config.dv_radial_range_avoidance[0]..self.config.dv_radial_range_avoidance[1]),
            dv_along: rng.gen_range(self.config.dv_along_range_avoidance[0]..self.config.dv_along_range_avoidance[1]),
            dv_cross: rng.gen_range(self.config.dv_cross_range_avoidance[0]..self.config.dv_cross_range_avoidance[1]),
            fitness: 0.0,
        }
    }

    fn tournament_select<'a>(
        &self,
        pop: &'a [Individual],
        rng: &mut rand::rngs::ThreadRng,
    ) -> &'a Individual {
        let mut best = &pop[rng.gen_range(0..pop.len())];
        for _ in 1..self.config.tournament_k {
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
        rng: &mut rand::rngs::ThreadRng,
    ) -> Individual {
        let alpha = self.config.blx_alpha;
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
        if rng.gen::<f64>() < self.config.mutation_rate {
            let delta = (self.config.dv_radial_range_station[1] - self.config.dv_radial_range_station[0]) * 0.2;
            ind.dv_radial += rng.gen_range(-delta..delta);
        }
        if rng.gen::<f64>() < self.config.mutation_rate {
            let delta = (self.config.dv_along_range_station[1] - self.config.dv_along_range_station[0]) * 0.2;
            ind.dv_along += rng.gen_range(-delta..delta);
        }
        if rng.gen::<f64>() < self.config.mutation_rate {
            let delta = (self.config.dv_cross_range_station[1] - self.config.dv_cross_range_station[0]) * 0.2;
            ind.dv_cross += rng.gen_range(-delta..delta);
        }
    }

    fn mutate_avoidance(&self, ind: &mut Individual, rng: &mut rand::rngs::ThreadRng) {
        if rng.gen::<f64>() < self.config.mutation_rate {
            let delta = (self.config.dv_radial_range_avoidance[1] - self.config.dv_radial_range_avoidance[0]) * 0.2;
            ind.dv_radial += rng.gen_range(-delta..delta);
        }
        if rng.gen::<f64>() < self.config.mutation_rate {
            let delta = (self.config.dv_along_range_avoidance[1] - self.config.dv_along_range_avoidance[0]) * 0.2;
            ind.dv_along += rng.gen_range(-delta..delta);
        }
        if rng.gen::<f64>() < self.config.mutation_rate {
            let delta = (self.config.dv_cross_range_avoidance[1] - self.config.dv_cross_range_avoidance[0]) * 0.2;
            ind.dv_cross += rng.gen_range(-delta..delta);
        }
    }

    fn migrate(&self, islands: &mut [Island]) {
        let migrants: Vec<Vec<Individual>> = islands
            .iter()
            .map(|island| {
                let mut sorted = island.population.clone();
                sorted.sort_by(|a, b| b.fitness.partial_cmp(&a.fitness).unwrap());
                sorted[..self.config.migration_count.min(sorted.len())].to_vec()
            })
            .collect();
        for i in 0..islands.len() {
            let target = (i + 1) % islands.len();
            let count = migrants[i].len().min(islands[target].population.len());
            if count > 0 {
                islands[target]
                    .population
                    .sort_by(|a, b| a.fitness.partial_cmp(&b.fitness).unwrap());
                for j in 0..count {
                    islands[target].population[j] = migrants[i][j].clone();
                }
                islands[target].best_fitness =
                    self.best_individual(&islands[target].population).fitness;
            }
        }
    }

    fn best_individual<'a>(&self, pop: &'a [Individual]) -> &'a Individual {
        pop.iter().max_by(|a, b| a.fitness.partial_cmp(&b.fitness).unwrap()).unwrap()
    }
}

pub type GeneticOrbitOptimizer = CoEvolutionOrbitOptimizer;

pub struct AlertManager {
    ground_station: GroundStationConfig,
}

impl AlertManager {
    pub fn new(config: GroundStationConfig) -> Self {
        Self {
            ground_station: config,
        }
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
            .post(&self.ground_station.alert_url)
            .json(alert)
            .timeout(std::time::Duration::from_secs(self.ground_station.push_timeout_seconds))
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
            .post(&self.ground_station.maneuver_url)
            .json(maneuver)
            .timeout(std::time::Duration::from_secs(self.ground_station.push_timeout_seconds))
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

pub enum OptimizerRequest {
    StationKeeping {
        telemetry: TelemetryData,
        target_sma: f64,
        reply: oneshot::Sender<ManeuverPlan>,
    },
    ConstellationStationKeeping {
        telemetry_map: HashMap<u16, TelemetryData>,
        target_sma: f64,
        reply: oneshot::Sender<Vec<ManeuverPlan>>,
    },
    AvoidanceManeuver {
        telemetry: TelemetryData,
        tle1: TleData,
        tle2: TleData,
        reply: oneshot::Sender<ManeuverPlan>,
    },
}

pub struct OrbitOptimizerService {
    optimizer: CoEvolutionOrbitOptimizer,
    alert_manager: AlertManager,
    propagator: Sgp4Propagator,
}

impl OrbitOptimizerService {
    pub fn new(config: &AppConfig) -> Self {
        let optimizer = CoEvolutionOrbitOptimizer::new(
            config.optimizer.clone(),
            config.atmosphere.clone(),
        );
        let alert_manager = AlertManager::new(config.ground_station.clone());
        let propagator = Sgp4Propagator::new(&config.sgp4);
        Self {
            optimizer,
            alert_manager,
            propagator,
        }
    }

    pub async fn run(mut self, mut request_rx: mpsc::Receiver<OptimizerRequest>) {
        while let Some(req) = request_rx.recv().await {
            match req {
                OptimizerRequest::StationKeeping { telemetry, target_sma, reply } => {
                    let plan = self.optimizer.optimize_station_keeping(&telemetry, target_sma);
                    let _ = reply.send(plan);
                }
                OptimizerRequest::ConstellationStationKeeping { telemetry_map, target_sma, reply } => {
                    let plans = self.optimizer.optimize_constellation_station_keeping(&telemetry_map, target_sma);
                    let _ = reply.send(plans);
                }
                OptimizerRequest::AvoidanceManeuver { telemetry, tle1, tle2, reply } => {
                    let plan = self.optimizer.optimize_avoidance_maneuver(&telemetry, &tle1, &tle2, &self.propagator);
                    let _ = reply.send(plan);
                }
            }
        }
    }
}
