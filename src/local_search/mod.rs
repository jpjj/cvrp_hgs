//! Local search operators for the HGS-CVRP algorithm.

pub mod relocate;
pub mod swap;
pub mod swap_star;
pub mod two_opt;
pub mod two_opt_star;
pub mod utils;

use crate::problem::Problem;
use crate::solution::{Route, Solution};
use std::collections::HashMap;
use std::f64;

use self::utils::RouteInfo;

/// Manages the local search phase of the HGS-CVRP algorithm.
pub struct LocalSearch {
    pub granularity: usize,
    /// Timestamp for route modifications, used for efficient move testing
    route_timestamps: Vec<usize>,
    /// Timestamp for move testing per customer
    move_timestamps: HashMap<(usize, usize, usize), usize>,
    /// Current move count, used as timestamp
    move_count: usize,
    /// SWAP* route polar sectors for pruning
    route_sectors: Vec<RouteInfo>,
    /// Preprocessed neighbors for each customer
    customer_neighbors: HashMap<usize, Vec<usize>>,
}

impl LocalSearch {
    /// Create a new local search instance.
    pub fn new(granularity: usize) -> Self {
        LocalSearch {
            granularity,
            route_timestamps: Vec::new(),
            move_timestamps: HashMap::new(),
            move_count: 0,
            route_sectors: Vec::new(),
            customer_neighbors: HashMap::new(),
        }
    }

    /// Run local search to improve a solution.
    pub fn educate(&mut self, solution: &mut Solution, problem: &Problem, capacity_penalty: f64) {
        // Initialize our tracking structures
        self.initialize_tracking(solution);

        // Preprocess neighbors if not already done
        if self.customer_neighbors.is_empty() {
            self.preprocess_neighbors(problem);
        }

        // Initial evaluation
        solution.evaluate(problem, capacity_penalty);

        // Main local search loop
        let mut improvement = true;
        while improvement {
            improvement = false;

            // Try all neighborhoods
            improvement |= self.relocate_neighborhood(solution, problem, capacity_penalty);
            improvement |= self.swap_neighborhood(solution, problem, capacity_penalty);
            improvement |= self.two_opt_neighborhood(solution, problem, capacity_penalty);
            improvement |= self.two_opt_star_neighborhood(solution, problem, capacity_penalty);
            improvement |= self.swap_star_neighborhood(solution, problem, capacity_penalty);
        }
    }

    /// Preprocess neighbors for all customers based on granularity.
    /// This significantly improves performance by avoiding repeated distance calculations.
    fn preprocess_neighbors(&mut self, problem: &Problem) {
        self.customer_neighbors.clear();

        // For each customer (excluding depot)
        for i in 0..problem.nodes.len() {
            if i != problem.depot_index {
                // Calculate and store its neighbors
                let neighbors = utils::get_neighbors(i, problem, self.granularity);
                self.customer_neighbors.insert(i, neighbors);
            }
        }
    }

    /// Initialize the tracking structures for the local search.
    fn initialize_tracking(&mut self, solution: &Solution) {
        self.route_timestamps = vec![0; solution.routes.len()];
        self.move_count = 0;
        self.move_timestamps.clear();
        self.route_sectors.clear();
    }

    /// Update timestamps when a route is modified.
    pub(crate) fn update_route_timestamp(&mut self, route_idx: usize) {
        self.move_count += 1;
        self.route_timestamps[route_idx] = self.move_count;
    }

    /// Check if a move has been tested before and is still valid.
    pub(crate) fn is_move_valid(
        &mut self,
        customer: usize,
        move_type: usize,
        route_idx: usize,
    ) -> bool {
        let key = (customer, move_type, route_idx);
        let route_ts = self.route_timestamps[route_idx];

        if let Some(move_ts) = self.move_timestamps.get(&key) {
            // If the move timestamp is more recent than the route timestamp,
            // this move has been tested after the last route modification
            if *move_ts > route_ts {
                return false;
            }
        }

        // Update the move timestamp
        self.move_count += 1;
        self.move_timestamps.insert(key, self.move_count);
        true
    }

    /// Try to repair an infeasible solution.
    pub fn repair(&mut self, solution: &mut Solution, problem: &Problem) {
        // Use local search with much higher capacity penalty
        let high_penalty = if solution.excess_capacity > 0.0 {
            10.0 * solution.cost / solution.excess_capacity
        } else {
            1000.0 // High default value if excess is 0
        };

        // Run local search with high penalty to focus on removing capacity violations
        self.educate(solution, problem, high_penalty);
    }
}
