//! Configuration parameters for the HGS-CVRP algorithm.

use serde::{Deserialize, Serialize};
use std::time::Duration;

/// Configuration settings for the HGS-CVRP algorithm.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    /// Minimum population size (μ)
    pub min_pop_size: usize,
    /// Number of individuals in a generation (λ)
    pub generation_size: usize,
    /// Number of elite individuals considered in fitness calculation
    pub n_elite: usize,
    /// Number of closest solutions considered in diversity calculation
    pub n_closest: usize,
    /// Granularity parameter for local search neighborhoods
    pub granularity: usize,
    /// Target proportion of feasible individuals
    pub target_feasible_ratio: f64,
    /// Initial penalty coefficient for capacity violations
    pub initial_capacity_penalty: f64,
    /// Maximum number of iterations without improvement
    pub max_iterations_without_improvement: u32,
    /// Optional time limit for the algorithm
    pub time_limit: Option<Duration>,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            min_pop_size: 25,
            generation_size: 40,
            n_elite: 4,
            n_closest: 5,
            granularity: 20,
            target_feasible_ratio: 0.2,
            initial_capacity_penalty: 1.0,
            max_iterations_without_improvement: 20000,
            time_limit: None,
        }
    }
}

impl Config {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Config::default()
    }

    /// Set the minimum population size.
    pub fn with_min_pop_size(mut self, size: usize) -> Self {
        self.min_pop_size = size;
        self
    }

    /// Set the generation size.
    pub fn with_generation_size(mut self, size: usize) -> Self {
        self.generation_size = size;
        self
    }

    /// Set the number of elite individuals.
    pub fn with_n_elite(mut self, n: usize) -> Self {
        self.n_elite = n;
        self
    }

    /// Set the number of closest solutions for diversity calculation.
    pub fn with_n_closest(mut self, n: usize) -> Self {
        self.n_closest = n;
        self
    }

    /// Set the granularity parameter.
    pub fn with_granularity(mut self, g: usize) -> Self {
        self.granularity = g;
        self
    }

    /// Set the target ratio of feasible individuals.
    pub fn with_target_feasible_ratio(mut self, ratio: f64) -> Self {
        self.target_feasible_ratio = ratio;
        self
    }

    /// Set the initial capacity penalty.
    pub fn with_initial_capacity_penalty(mut self, penalty: f64) -> Self {
        self.initial_capacity_penalty = penalty;
        self
    }

    /// Set the maximum iterations without improvement.
    pub fn with_max_iterations_without_improvement(mut self, iterations: u32) -> Self {
        self.max_iterations_without_improvement = iterations;
        self
    }

    /// Set the time limit.
    pub fn with_time_limit(mut self, duration: Duration) -> Self {
        self.time_limit = Some(duration);
        self
    }
}
