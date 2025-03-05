//! 2-Opt* neighborhood for local search (inter-route).

use crate::problem::Problem;
use crate::solution::Solution;
use rand::seq::SliceRandom;
use rand::thread_rng;
use std::f64;

use super::utils::{find_route_for_customer, get_neighbors};
use super::LocalSearch;

impl LocalSearch {
    /// Implement the 2-Opt* neighborhood for inter-route improvements.
    pub fn two_opt_star_neighborhood(
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

        for r1_pos in 0..route_indices.len() {
            let r1_idx = route_indices[r1_pos];
            let r1 = &solution.routes[r1_idx].clone();

            if r1.is_empty() {
                continue;
            }

            for r2_pos in r1_pos + 1..route_indices.len() {
                let r2_idx = route_indices[r2_pos];
                let r2 = &solution.routes[r2_idx].clone();

                if r2.is_empty() {
                    continue;
                }

                // Try cutting each route at different positions
                for i in 0..r1.customers.len() {
                    let customer1 = r1.customers[i];

                    // Use preprocessed neighbors
                    let neighbors = &self.customer_neighbors[&customer1].clone();

                    for &neighbor in neighbors {
                        // Find this neighbor in r2
                        if let Some(j) = r2.customers.iter().position(|&c| c == neighbor) {
                            // Check if this move has been tested before
                            if !self.is_move_valid(customer1, 3, r2_idx) {
                                continue;
                            }

                            // Evaluate 2-Opt* move
                            let delta = self.evaluate_two_opt_star(
                                solution,
                                problem,
                                r1_idx,
                                r2_idx,
                                i,
                                j,
                                capacity_penalty,
                            );

                            if delta < -1e-6 {
                                // Apply the move
                                self.apply_two_opt_star(solution, r1_idx, r2_idx, i, j);

                                // Update route timestamps
                                self.update_route_timestamp(r1_idx);
                                self.update_route_timestamp(r2_idx);

                                // Re-evaluate the solution
                                solution.evaluate(problem, capacity_penalty);

                                improvement = true;
                                break;
                            }
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

            if improvement {
                break;
            }
        }

        improvement
    }

    /// Evaluate a 2-Opt* move (exchanging tails between two routes).
    fn evaluate_two_opt_star(
        &self,
        solution: &Solution,
        problem: &Problem,
        r1_idx: usize,
        r2_idx: usize,
        i: usize,
        j: usize,
        capacity_penalty: f64,
    ) -> f64 {
        let r1 = &solution.routes[r1_idx];
        let r2 = &solution.routes[r2_idx];

        // Get customers at the cutting points
        let customer1 = r1.customers[i];
        let customer2 = r2.customers[j];

        // Calculate new loads
        let r1_tail_load: f64 = r1
            .customers
            .iter()
            .skip(i + 1)
            .map(|&c| problem.nodes[c].demand)
            .sum();
        let r2_tail_load: f64 = r2
            .customers
            .iter()
            .skip(j + 1)
            .map(|&c| problem.nodes[c].demand)
            .sum();

        let r1_new_load = r1.load - r1_tail_load + r2_tail_load;
        let r2_new_load = r2.load - r2_tail_load + r1_tail_load;

        // Calculate distance changes
        let next1 = if i + 1 < r1.customers.len() {
            r1.customers[i + 1]
        } else {
            problem.depot_index
        };
        let next2 = if j + 1 < r2.customers.len() {
            r2.customers[j + 1]
        } else {
            problem.depot_index
        };

        // Old connections
        let old_dist =
            problem.get_distance(customer1, next1) + problem.get_distance(customer2, next2);

        // New connections
        let new_dist =
            problem.get_distance(customer1, next2) + problem.get_distance(customer2, next1);

        let distance_delta = new_dist - old_dist;

        // Calculate capacity penalties
        let r1_original_excess = (r1.load - problem.vehicle_capacity).max(0.0);
        let r1_new_excess = (r1_new_load - problem.vehicle_capacity).max(0.0);
        let r1_penalty_delta = capacity_penalty * (r1_new_excess - r1_original_excess);

        let r2_original_excess = (r2.load - problem.vehicle_capacity).max(0.0);
        let r2_new_excess = (r2_new_load - problem.vehicle_capacity).max(0.0);
        let r2_penalty_delta = capacity_penalty * (r2_new_excess - r2_original_excess);

        // Total cost change
        distance_delta + r1_penalty_delta + r2_penalty_delta
    }

    /// Apply a 2-Opt* move.
    fn apply_two_opt_star(
        &mut self,
        solution: &mut Solution,
        r1_idx: usize,
        r2_idx: usize,
        i: usize,
        j: usize,
    ) {
        // Get tails
        let r1_tail: Vec<usize> = solution.routes[r1_idx].customers.drain(i + 1..).collect();
        let r2_tail: Vec<usize> = solution.routes[r2_idx].customers.drain(j + 1..).collect();

        // Swap tails
        solution.routes[r1_idx].customers.extend(r2_tail);
        solution.routes[r2_idx].customers.extend(r1_tail);

        // Mark routes as modified
        solution.routes[r1_idx].modified = true;
        solution.routes[r2_idx].modified = true;
    }
}
