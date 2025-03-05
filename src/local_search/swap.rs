//! Swap neighborhood for local search.

use crate::problem::Problem;
use crate::solution::{Route, Solution};
use rand::seq::SliceRandom;
use rand::thread_rng;
use std::f64;

use super::utils::{find_route_for_customer, get_neighbors};
use super::LocalSearch;

impl LocalSearch {
    /// Implement the Swap neighborhood.
    pub fn swap_neighborhood(
        &mut self,
        solution: &mut Solution,
        problem: &Problem,
        capacity_penalty: f64,
    ) -> bool {
        let mut improvement = false;
        let mut rng = thread_rng();

        // Consider all pairs of routes
        let routes = solution.routes.len();
        let mut route_indices: Vec<usize> = (0..routes).collect();
        route_indices.shuffle(&mut rng);

        for &r1_idx in &route_indices {
            let r1 = &solution.routes[r1_idx].clone();

            if r1.is_empty() {
                continue;
            }

            // Try to swap each customer in r1
            let customers = r1.customers.len();
            let mut customer_indices: Vec<usize> = (0..customers).collect();
            customer_indices.shuffle(&mut rng);

            for &c1_pos in &customer_indices {
                let customer1 = r1.customers[c1_pos];

                // Use preprocessed neighbors
                let neighbors = &self.customer_neighbors[&customer1].clone();

                for &neighbor in neighbors {
                    // Find which route contains this neighbor
                    let r2_idx = find_route_for_customer(solution, neighbor);

                    if r2_idx.is_none() || r2_idx.unwrap() == r1_idx {
                        continue;
                    }

                    let r2_idx = r2_idx.unwrap();
                    let r2 = &solution.routes[r2_idx];

                    // Find the position of the neighbor in r2
                    let c2_pos = r2.customers.iter().position(|&c| c == neighbor).unwrap();

                    // Check if this move has been tested before
                    if !self.is_move_valid(customer1, 1, r2_idx) {
                        continue;
                    }

                    // Evaluate the Swap move
                    let delta = self.evaluate_swap(
                        solution,
                        problem,
                        r1_idx,
                        r2_idx,
                        c1_pos,
                        c2_pos,
                        capacity_penalty,
                    );

                    if delta < -1e-6 {
                        // Apply the move
                        self.apply_swap(solution, r1_idx, r2_idx, c1_pos, c2_pos);

                        // Update route timestamps
                        self.update_route_timestamp(r1_idx);
                        self.update_route_timestamp(r2_idx);

                        // Re-evaluate the solution
                        solution.evaluate(problem, capacity_penalty);

                        improvement = true;
                        break;
                    }
                }

                if improvement {
                    break;
                }
            }

            if improvement {
                break;
            }
        }

        improvement
    }

    /// Evaluate a Swap move (exchanging two customers between different routes).
    fn evaluate_swap(
        &self,
        solution: &Solution,
        problem: &Problem,
        r1_idx: usize,
        r2_idx: usize,
        c1_pos: usize,
        c2_pos: usize,
        capacity_penalty: f64,
    ) -> f64 {
        let r1 = &solution.routes[r1_idx];
        let r2 = &solution.routes[r2_idx];
        let customer1 = r1.customers[c1_pos];
        let customer2 = r2.customers[c2_pos];

        // Calculate distance changes
        let r1_delta = self.calculate_swap_cost_for_route(r1, c1_pos, customer2, problem);
        let r2_delta = self.calculate_swap_cost_for_route(r2, c2_pos, customer1, problem);

        // Calculate load changes
        let demand1 = problem.nodes[customer1].demand;
        let demand2 = problem.nodes[customer2].demand;

        let r1_new_load = r1.load - demand1 + demand2;
        let r2_new_load = r2.load - demand2 + demand1;

        // Calculate capacity penalties
        let r1_original_excess = (r1.load - problem.vehicle_capacity).max(0.0);
        let r1_new_excess = (r1_new_load - problem.vehicle_capacity).max(0.0);
        let r1_penalty_delta = capacity_penalty * (r1_new_excess - r1_original_excess);

        let r2_original_excess = (r2.load - problem.vehicle_capacity).max(0.0);
        let r2_new_excess = (r2_new_load - problem.vehicle_capacity).max(0.0);
        let r2_penalty_delta = capacity_penalty * (r2_new_excess - r2_original_excess);

        // Total cost change
        r1_delta + r2_delta + r1_penalty_delta + r2_penalty_delta
    }

    /// Calculate the cost change when swapping a customer in a route.
    fn calculate_swap_cost_for_route(
        &self,
        route: &Route,
        pos: usize,
        new_customer: usize,
        problem: &Problem,
    ) -> f64 {
        let customers = &route.customers;
        let n = customers.len();

        if n <= 1 {
            // Special case: single-customer route
            return 0.0;
        }

        // Calculate the change in distance
        let prev_idx = if pos > 0 {
            customers[pos - 1]
        } else {
            problem.depot_index
        };
        let curr_idx = customers[pos];
        let next_idx = if pos < n - 1 {
            customers[pos + 1]
        } else {
            problem.depot_index
        };

        // Remove current connections
        let old_distance =
            problem.get_distance(prev_idx, curr_idx) + problem.get_distance(curr_idx, next_idx);

        // Add new connections
        let new_distance = problem.get_distance(prev_idx, new_customer)
            + problem.get_distance(new_customer, next_idx);

        // The delta is the new distance minus the old distance
        new_distance - old_distance
    }

    /// Apply a Swap move.
    fn apply_swap(
        &mut self,
        solution: &mut Solution,
        r1_idx: usize,
        r2_idx: usize,
        c1_pos: usize,
        c2_pos: usize,
    ) {
        // Swap the customers
        let temp = solution.routes[r1_idx].customers[c1_pos];
        solution.routes[r1_idx].customers[c1_pos] = solution.routes[r2_idx].customers[c2_pos];
        solution.routes[r2_idx].customers[c2_pos] = temp;

        // Mark routes as modified
        solution.routes[r1_idx].modified = true;
        solution.routes[r2_idx].modified = true;
    }
}
