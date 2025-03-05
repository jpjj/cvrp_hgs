//! 2-Opt neighborhood for local search (intra-route).

use crate::problem::Problem;
use crate::solution::Solution;
use rand::seq::SliceRandom;
use rand::thread_rng;
use std::f64;

use super::LocalSearch;

impl LocalSearch {
    /// Implement the 2-Opt neighborhood for intra-route improvements.
    pub fn two_opt_neighborhood(
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

        for &r_idx in &route_indices {
            let route = &solution.routes[r_idx].clone();

            if route.customers.len() < 4 {
                // Need at least 4 customers for 2-opt to be effective
                continue;
            }

            // Try all pairs of edges
            let n = route.customers.len();
            let mut positions: Vec<usize> = (0..n - 1).collect();
            positions.shuffle(&mut rng);

            for &i in &positions {
                let mut positions_j: Vec<usize> = (i + 2..n).collect();
                positions_j.shuffle(&mut rng);

                for &j in &positions_j {
                    // Check if this move has been tested before
                    if !self.is_move_valid(route.customers[i], 2, r_idx) {
                        continue;
                    }

                    // Evaluate 2-opt move
                    let delta = self.evaluate_two_opt(solution, problem, r_idx, i, j);

                    if delta < -1e-6 {
                        // Apply the move
                        self.apply_two_opt(solution, r_idx, i, j);

                        // Update route timestamp
                        self.update_route_timestamp(r_idx);

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

    /// Evaluate a 2-Opt move (reversing a segment of a route).
    fn evaluate_two_opt(
        &self,
        solution: &Solution,
        problem: &Problem,
        r_idx: usize,
        i: usize,
        j: usize,
    ) -> f64 {
        let route = &solution.routes[r_idx];
        let customers = &route.customers;

        // Get indices (with special case for depot)
        let i_node = customers[i];
        let i_next = customers[i + 1];
        let j_node = customers[j];
        let j_next = if j + 1 < customers.len() {
            customers[j + 1]
        } else {
            problem.depot_index
        };

        // Calculate old edge costs
        let old_cost = problem.get_distance(i_node, i_next) + problem.get_distance(j_node, j_next);

        // Calculate new edge costs after 2-opt
        let new_cost = problem.get_distance(i_node, j_node) + problem.get_distance(i_next, j_next);

        // Return delta
        new_cost - old_cost
    }

    /// Apply a 2-Opt move.
    fn apply_two_opt(&mut self, solution: &mut Solution, r_idx: usize, i: usize, j: usize) {
        // Reverse the segment from i+1 to j
        let route = &mut solution.routes[r_idx];
        let segment = &mut route.customers[i + 1..=j];
        segment.reverse();

        // Mark route as modified
        route.modified = true;
    }
}
