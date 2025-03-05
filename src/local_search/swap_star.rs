//! SWAP* neighborhood for local search.

use crate::problem::Problem;
use crate::solution::{Route, Solution};
use std::f64;

use super::utils::{
    calculate_insertion_cost, calculate_removal_cost, create_temp_route, get_neighbors, RouteInfo,
};
use super::LocalSearch;

impl LocalSearch {
    /// Implement the SWAP* neighborhood.
    pub fn swap_star_neighborhood(
        &mut self,
        solution: &mut Solution,
        problem: &Problem,
        capacity_penalty: f64,
    ) -> bool {
        let mut improvement = false;

        // First, calculate route polar sectors for pruning
        self.calculate_route_sectors(solution, problem);

        // Consider all pairs of routes with intersecting polar sectors
        for r1_idx in 0..solution.routes.len() {
            let r1 = &solution.routes[r1_idx].clone();

            if r1.is_empty() {
                continue;
            }

            let r1_info = &self.route_sectors[r1_idx].clone();

            for r2_idx in 0..solution.routes.len() {
                if r1_idx == r2_idx {
                    continue;
                }

                let r2 = &solution.routes[r2_idx].clone();

                if r2.is_empty() {
                    continue;
                }

                let r2_info = &self.route_sectors[r2_idx];

                // Check if route sectors intersect (for pruning)
                if !self.sectors_intersect(r1_info, r2_info) {
                    continue;
                }

                // For each customer in r1, try to swap with each customer in r2
                for (pos1, &customer1) in r1.customers.iter().enumerate() {
                    // Check if this move has been tested before
                    if !self.is_move_valid(customer1, 4, r2_idx) {
                        continue;
                    }

                    // Preprocess: find top 3 best insertion positions in r2 for customer1
                    let top_positions_in_r2 =
                        self.find_top_insertion_positions(customer1, r2, problem);

                    for (pos2, &customer2) in r2.customers.iter().enumerate() {
                        // Preprocess: find top 3 best insertion positions in r1 for customer2
                        let top_positions_in_r1 =
                            self.find_top_insertion_positions(customer2, r1, problem);

                        // Evaluate the SWAP* move
                        let (delta, best_pos1, best_pos2) = self.evaluate_swap_star(
                            solution,
                            problem,
                            r1_idx,
                            r2_idx,
                            pos1,
                            pos2,
                            &top_positions_in_r1,
                            &top_positions_in_r2,
                            capacity_penalty,
                        );

                        if delta < -1e-6 {
                            // Apply the move
                            self.apply_swap_star(
                                solution, r1_idx, r2_idx, pos1, pos2, best_pos1, best_pos2,
                            );

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

            if improvement {
                break;
            }
        }

        improvement
    }

    /// Calculate route polar sectors for SWAP* pruning.
    fn calculate_route_sectors(&mut self, solution: &Solution, problem: &Problem) {
        self.route_sectors.clear();

        let depot_x = problem.nodes[problem.depot_index].x;
        let depot_y = problem.nodes[problem.depot_index].y;

        for (r_idx, route) in solution.routes.iter().enumerate() {
            if route.is_empty() {
                // Empty routes have no sector
                self.route_sectors.push(RouteInfo {
                    route_index: r_idx,
                    polar_min: 0.0,
                    polar_max: 0.0,
                });
                continue;
            }

            // Calculate polar angle for each customer in the route
            let mut min_angle = std::f64::consts::PI * 2.0;
            let mut max_angle: f64 = 0.0;

            for &customer in &route.customers {
                let node = &problem.nodes[customer];
                let dx = node.x - depot_x;
                let dy = node.y - depot_y;

                // Calculate polar angle (0 to 2π)
                let angle = dy.atan2(dx);
                let normalized_angle = if angle < 0.0 {
                    angle + 2.0 * std::f64::consts::PI
                } else {
                    angle
                };

                min_angle = min_angle.min(normalized_angle);
                max_angle = max_angle.max(normalized_angle);
            }

            // Handle cases where the sector crosses the 0/2π boundary
            if max_angle - min_angle > std::f64::consts::PI {
                // The sector crosses the boundary, adjust
                std::mem::swap(&mut min_angle, &mut max_angle);
                max_angle += 2.0 * std::f64::consts::PI;
            }

            self.route_sectors.push(RouteInfo {
                route_index: r_idx,
                polar_min: min_angle,
                polar_max: max_angle,
            });
        }
    }

    /// Check if two route sectors intersect.
    fn sectors_intersect(&self, s1: &RouteInfo, s2: &RouteInfo) -> bool {
        // Special case: empty routes
        if s1.polar_min == s1.polar_max || s2.polar_min == s2.polar_max {
            return false;
        }

        // Check for intersection
        !(s1.polar_max < s2.polar_min || s2.polar_max < s1.polar_min)
    }

    /// Find the top 3 best insertion positions for a customer in a route.
    fn find_top_insertion_positions(
        &self,
        customer: usize,
        route: &Route,
        problem: &Problem,
    ) -> Vec<(usize, f64)> {
        let mut positions = Vec::new();

        for i in 0..=route.customers.len() {
            let cost = calculate_insertion_cost(route, customer, i, problem);
            positions.push((i, cost));
        }

        // Sort by cost (ascending)
        positions.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

        // Take top 3 (or fewer if route is small)
        positions.truncate(3.min(positions.len()));
        positions
    }

    /// Evaluate a SWAP* move.
    fn evaluate_swap_star(
        &self,
        solution: &Solution,
        problem: &Problem,
        r1_idx: usize,
        r2_idx: usize,
        pos1: usize,
        pos2: usize,
        top_positions_in_r1: &[(usize, f64)],
        top_positions_in_r2: &[(usize, f64)],
        capacity_penalty: f64,
    ) -> (f64, usize, usize) {
        let r1 = &solution.routes[r1_idx];
        let r2 = &solution.routes[r2_idx];
        let customer1 = r1.customers[pos1];
        let customer2 = r2.customers[pos2];
        let demand1 = problem.nodes[customer1].demand;
        let demand2 = problem.nodes[customer2].demand;

        let mut best_delta = f64::INFINITY;
        let mut best_pos1 = 0;
        let mut best_pos2 = 0;

        // Calculate the cost of removing both customers
        let r1_remove_delta = calculate_removal_cost(r1, pos1, problem);
        let r2_remove_delta = calculate_removal_cost(r2, pos2, problem);

        // Try the best insertion positions and also the original positions
        let mut check_positions = Vec::new();

        // Customer1 in r2 (without customer2)
        for &(insert_pos, _) in top_positions_in_r2 {
            check_positions.push((insert_pos, insert_pos == pos2));
        }
        // Also check if customer1 can replace customer2 directly
        if !check_positions.iter().any(|&(_, is_original)| is_original) {
            check_positions.push((pos2, true));
        }

        for (insert_pos2, is_original_pos2) in check_positions.drain(..) {
            // Customer2 in r1 (without customer1)
            let mut check_positions_r1 = Vec::new();

            for &(insert_pos, _) in top_positions_in_r1 {
                check_positions_r1.push((insert_pos, insert_pos == pos1));
            }
            // Also check if customer2 can replace customer1 directly
            if !check_positions_r1
                .iter()
                .any(|&(_, is_original)| is_original)
            {
                check_positions_r1.push((pos1, true));
            }

            for (insert_pos1, is_original_pos1) in check_positions_r1 {
                // Calculate r1 delta (remove customer1, insert customer2)
                let r1_insert_pos = if insert_pos1 > pos1 {
                    insert_pos1 - 1
                } else {
                    insert_pos1
                };
                let r1_temp = create_temp_route(r1, pos1, customer2, r1_insert_pos);
                let r1_delta = r1_temp.distance - r1.distance;

                // Calculate r2 delta (remove customer2, insert customer1)
                let r2_insert_pos = if insert_pos2 > pos2 {
                    insert_pos2 - 1
                } else {
                    insert_pos2
                };
                let r2_temp = create_temp_route(r2, pos2, customer1, r2_insert_pos);
                let r2_delta = r2_temp.distance - r2.distance;

                // Calculate load changes
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
                let total_delta = r1_delta + r2_delta + r1_penalty_delta + r2_penalty_delta;

                if total_delta < best_delta {
                    best_delta = total_delta;
                    best_pos1 = insert_pos1;
                    best_pos2 = insert_pos2;
                }
            }
        }

        (best_delta, best_pos1, best_pos2)
    }

    /// Apply a SWAP* move.
    fn apply_swap_star(
        &mut self,
        solution: &mut Solution,
        r1_idx: usize,
        r2_idx: usize,
        pos1: usize,
        pos2: usize,
        insert_pos1: usize,
        insert_pos2: usize,
    ) {
        // Remove customers from their original routes
        let customer1 = solution.routes[r1_idx].customers.remove(pos1);
        let customer2 = solution.routes[r2_idx].customers.remove(pos2);

        // Insert customers at their new positions
        // Adjust positions if needed due to removed customers
        let adjusted_pos1 = if insert_pos1 > pos1 {
            insert_pos1 - 1
        } else {
            insert_pos1
        };
        let adjusted_pos2 = if insert_pos2 > pos2 {
            insert_pos2 - 1
        } else {
            insert_pos2
        };

        solution.routes[r1_idx]
            .customers
            .insert(adjusted_pos1, customer2);
        solution.routes[r2_idx]
            .customers
            .insert(adjusted_pos2, customer1);

        // Mark routes as modified
        solution.routes[r1_idx].modified = true;
        solution.routes[r2_idx].modified = true;
    }
}
