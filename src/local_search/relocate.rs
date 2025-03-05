//! Relocate neighborhood for local search.

use crate::problem::Problem;
use crate::solution::Solution;
use rand::seq::SliceRandom;
use rand::thread_rng;
use std::f64;

use super::utils::{
    self, calculate_insertion_cost, calculate_removal_cost, find_route_for_customer, get_neighbors,
};
use super::LocalSearch;

impl LocalSearch {
    /// Implement the Relocate neighborhood.
    pub fn relocate_neighborhood(
        &mut self,
        solution: &mut Solution,
        problem: &Problem,
        capacity_penalty: f64,
    ) -> bool {
        let mut improvement = false;
        let mut rng = thread_rng();

        // Consider all routes
        let routes = solution.routes.len();
        let mut route_indices: Vec<usize> = (0..routes).collect();
        route_indices.shuffle(&mut rng);

        for &r1_idx in &route_indices {
            let r1 = &solution.routes[r1_idx].clone();

            if r1.is_empty() {
                continue;
            }

            // Try to relocate each customer
            let customers = r1.customers.len();
            let mut customer_indices: Vec<usize> = (0..customers).collect();
            customer_indices.shuffle(&mut rng);

            for &c_pos in &customer_indices {
                let customer = r1.customers[c_pos];

                // Use preprocessed neighbors instead of recalculating them
                let maybe_neighbors = self.customer_neighbors.get(&customer);
                let neighbors = match maybe_neighbors {
                    Some(neighbors) => neighbors.clone(),
                    None => {
                        let neighbors = utils::get_neighbors(customer, problem, self.granularity);
                        self.customer_neighbors.insert(customer, neighbors.clone());
                        neighbors
                    }
                };

                for &neighbor in &neighbors {
                    // Find which route contains this neighbor
                    let r2_idx = find_route_for_customer(solution, neighbor);

                    if r2_idx.is_none() || r2_idx.unwrap() == r1_idx {
                        continue;
                    }

                    let r2_idx = r2_idx.unwrap();

                    // Check if this move has been tested before
                    if !self.is_move_valid(customer, 0, r2_idx) {
                        continue;
                    }

                    // Evaluate the Relocate move
                    let (delta, insert_pos) = self.evaluate_relocate(
                        solution,
                        problem,
                        r1_idx,
                        r2_idx,
                        c_pos,
                        capacity_penalty,
                    );

                    if delta < -1e-6 {
                        // Apply the move
                        self.apply_relocate(solution, r1_idx, r2_idx, c_pos, insert_pos);

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

    /// Evaluate a Relocate move (moving a customer from one route to another).
    fn evaluate_relocate(
        &self,
        solution: &Solution,
        problem: &Problem,
        r1_idx: usize,
        r2_idx: usize,
        c_pos: usize,
        capacity_penalty: f64,
    ) -> (f64, usize) {
        let r1 = &solution.routes[r1_idx];
        let r2 = &solution.routes[r2_idx];
        let customer = r1.customers[c_pos];
        let demand = problem.nodes[customer].demand;

        // Check if removing customer from r1 makes it empty
        if r1.customers.len() == 1 {
            // Removing the only customer makes r1 empty
            let r1_cost = r1.distance;

            // Find best insertion position in r2
            let mut best_delta = f64::INFINITY;
            let mut best_pos = 0;

            for i in 0..=r2.customers.len() {
                let new_distance = calculate_insertion_cost(r2, customer, i, problem);
                let delta = new_distance - r2.distance;

                // Include capacity considerations
                let new_load = r2.load + demand;
                let original_excess = (r2.load - problem.vehicle_capacity).max(0.0);
                let new_excess = (new_load - problem.vehicle_capacity).max(0.0);
                let penalty_delta = capacity_penalty * (new_excess - original_excess);

                let total_delta = delta + penalty_delta;

                if total_delta < best_delta {
                    best_delta = total_delta;
                    best_pos = i;
                }
            }

            // Total cost change
            return (best_delta - r1_cost, best_pos);
        }

        // Normal case: r1 will still have customers after removal
        let r1_delta = calculate_removal_cost(r1, c_pos, problem);

        // Check load changes for r1
        let r1_new_load = r1.load - demand;
        let r1_original_excess = (r1.load - problem.vehicle_capacity).max(0.0);
        let r1_new_excess = (r1_new_load - problem.vehicle_capacity).max(0.0);
        let r1_penalty_delta = capacity_penalty * (r1_new_excess - r1_original_excess);

        // Find best insertion position in r2
        let mut best_delta = f64::INFINITY;
        let mut best_pos = 0;

        for i in 0..=r2.customers.len() {
            let r2_delta = calculate_insertion_cost(r2, customer, i, problem) - r2.distance;

            // Check load changes for r2
            let r2_new_load = r2.load + demand;
            let r2_original_excess = (r2.load - problem.vehicle_capacity).max(0.0);
            let r2_new_excess = (r2_new_load - problem.vehicle_capacity).max(0.0);
            let r2_penalty_delta = capacity_penalty * (r2_new_excess - r2_original_excess);

            let total_delta = r1_delta + r1_penalty_delta + r2_delta + r2_penalty_delta;

            if total_delta < best_delta {
                best_delta = total_delta;
                best_pos = i;
            }
        }

        (best_delta, best_pos)
    }

    /// Apply a Relocate move.
    fn apply_relocate(
        &mut self,
        solution: &mut Solution,
        r1_idx: usize,
        r2_idx: usize,
        c_pos: usize,
        insert_pos: usize,
    ) {
        // Remove customer from r1
        let customer = solution.routes[r1_idx].customers.remove(c_pos);

        // Insert into r2
        solution.routes[r2_idx]
            .customers
            .insert(insert_pos, customer);

        // Mark routes as modified
        solution.routes[r1_idx].modified = true;
        solution.routes[r2_idx].modified = true;
    }
}
