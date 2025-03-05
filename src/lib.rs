//! # HGS-CVRP
//!
//! A Rust implementation of the Hybrid Genetic Search algorithm for the
//! Capacitated Vehicle Routing Problem (CVRP).
//!
//! Based on the paper "Hybrid Genetic Search for the CVRP: Open-Source Implementation
//! and SWAP* Neighborhood" by Thibaut Vidal.
//!
//! The algorithm combines genetic search with local improvement heuristics and
//! strategic management of population diversity to efficiently solve CVRP instances.

pub mod config;
pub mod genetic;
pub mod individual;
pub mod local_search;
pub mod population;
pub mod problem;
pub mod solution;
pub mod split;
pub mod utils;

use crate::config::Config;
use crate::genetic::Genetic;
use crate::local_search::LocalSearch;
use crate::population::Population;
use crate::problem::Problem;
use crate::solution::Solution;
use crate::split::Split;

use std::time::{Duration, Instant};

/// The main algorithm structure that orchestrates the hybrid genetic search.
pub struct HgsAlgorithm {
    pub problem: Problem,
    pub population: Population,
    pub config: Config,
    pub best_solution: Option<Solution>,
    pub run_time: Duration,
    pub iterations: u32,
    pub iterations_without_improvement: u32,
    pub genetic: Genetic,
    pub split: Split,
    pub local_search: LocalSearch,
    pub start_time: Instant,
}

impl HgsAlgorithm {
    /// Create a new HGS instance for the given problem and configuration.
    pub fn new(problem: Problem, config: Config) -> Self {
        HgsAlgorithm {
            problem,
            population: Population::new(&config),
            config,
            best_solution: None,
            run_time: Duration::from_secs(0),
            iterations: 0,
            iterations_without_improvement: 0,
            genetic: Genetic,
            split: Split,
            local_search: LocalSearch::new(config.granularity),
            start_time: Instant::now(),
        }
    }

    /// Initialize the population with random solutions.
    pub fn initialize(&mut self) {
        self.population.initialize(&self.problem, &self.config);
        self.best_solution = self.population.get_best_feasible_solution().cloned();
    }

    /// Run the algorithm until the termination criteria are met.
    pub fn run(&mut self) -> &Solution {
        self.start_time = Instant::now();

        self.initialize();

        while !self.should_terminate() {
            // Select parents
            let (parent1, parent2) = self.population.select_parents();

            // Apply crossover to produce offspring
            let mut offspring = self.genetic.crossover(parent1, parent2);

            // Apply split algorithm to determine routes
            Split::split(&mut offspring, &self.problem);

            // Improve the offspring with local search
            self.local_search.educate(
                &mut offspring,
                &self.problem,
                self.population.capacity_penalty,
            );

            // Add the offspring to the population
            let previous_best = self.population.get_best_feasible_solution().cloned();
            self.population
                .insert_individual(Individual::new(offspring));

            // Update iteration counters
            self.iterations += 1;

            // Check if we have a new best solution
            let current_best = self.population.get_best_feasible_solution().cloned();

            if let (Some(prev), Some(curr)) = (previous_best, current_best.clone()) {
                if curr.cost < prev.cost {
                    self.best_solution = Some(curr);
                    self.iterations_without_improvement = 0;
                } else {
                    self.iterations_without_improvement += 1;
                }
            }

            // Manage population size if needed
            if self.population.should_manage_size() {
                self.population.select_survivors();
            }

            // Adjust penalty parameters
            self.population.adjust_penalties();
        }

        self.run_time = self.start_time.elapsed();
        self.best_solution.as_ref().unwrap()
    }

    /// Check if the termination criteria are met.
    fn should_terminate(&self) -> bool {
        // Terminate if we've reached max iterations without improvement
        if self.iterations_without_improvement >= self.config.max_iterations_without_improvement {
            return true;
        }

        // Terminate if we've reached the time limit
        if let Some(time_limit) = self.config.time_limit {
            if Instant::now().duration_since(self.start_time) >= time_limit {
                return true;
            }
        }

        false
    }
}
