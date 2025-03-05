//! Local search operators for the HGS-CVRP algorithm.

use crate::problem::Problem;
use crate::solution::{Route, Solution};
use rand::seq::SliceRandom;
use rand::thread_rng;
use std::collections::HashMap;
use std::f64;

/// A structure to hold route information for swap* neighborhood.
struct RouteInfo {
    pub route_index: usize,
    pub polar_min: f64,
    pub polar_max: f64,
}

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
        }
    }

    /// Run local search to improve a solution.
    pub fn educate(&mut self, solution: &mut Solution, problem: &Problem, capacity_penalty: f64) {
        // Initialize our tracking structures
        self.initialize_tracking(solution);

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

    /// Initialize the tracking structures for the local search.
    fn initialize_tracking(&mut self, solution: &Solution) {
        self.route_timestamps = vec![0; solution.routes.len()];
        self.move_count = 0;
        self.move_timestamps.clear();
        self.route_sectors.clear();
    }

    /// Update timestamps when a route is modified.
    fn update_route_timestamp(&mut self, route_idx: usize) {
        self.move_count += 1;
        self.route_timestamps[route_idx] = self.move_count;
    }

    /// Check if a move has been tested before and is still valid.
    fn is_move_valid(&mut self, customer: usize, move_type: usize, route_idx: usize) -> bool {
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

    /// Generate a list of neighbors for a customer based on granularity.
    fn get_neighbors(&self, customer: usize, problem: &Problem) -> Vec<usize> {
        let mut distances: Vec<(usize, f64)> = Vec::new();

        for i in 0..problem.nodes.len() {
            if i != customer && i != problem.depot_index {
                let dist = problem.get_distance(customer, i);
                distances.push((i, dist));
            }
        }

        // Sort by distance
        distances.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

        // Take the closest `granularity` neighbors
        let count = std::cmp::min(self.granularity, distances.len());
        distances.truncate(count);

        distances.into_iter().map(|(idx, _)| idx).collect()
    }

    /// Implement the Relocate neighborhood.
    fn relocate_neighborhood(
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
            let r1 = &solution.routes[r1_idx];

            if r1.is_empty() {
                continue;
            }

            // Try to relocate each customer
            let customers = r1.customers.len();
            let mut customer_indices: Vec<usize> = (0..customers).collect();
            customer_indices.shuffle(&mut rng);

            for &c_pos in &customer_indices {
                let customer = r1.customers[c_pos];

                // Only consider close routes based on granularity
                let neighbors = self.get_neighbors(customer, problem);

                for &neighbor in &neighbors {
                    // Find which route contains this neighbor
                    let r2_idx = self.find_route_for_customer(solution, neighbor);

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

    /// Find which route contains a specific customer.
    fn find_route_for_customer(&self, solution: &Solution, customer: usize) -> Option<usize> {
        for (idx, route) in solution.routes.iter().enumerate() {
            if route.customers.contains(&customer) {
                return Some(idx);
            }
        }
        None
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
                let new_distance = self.calculate_insertion_cost(r2, customer, i, problem);
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
        let r1_delta = self.calculate_removal_cost(r1, c_pos, problem);

        // Check load changes for r1
        let r1_new_load = r1.load - demand;
        let r1_original_excess = (r1.load - problem.vehicle_capacity).max(0.0);
        let r1_new_excess = (r1_new_load - problem.vehicle_capacity).max(0.0);
        let r1_penalty_delta = capacity_penalty * (r1_new_excess - r1_original_excess);

        // Find best insertion position in r2
        let mut best_delta = f64::INFINITY;
        let mut best_pos = 0;

        for i in 0..=r2.customers.len() {
            let r2_delta = self.calculate_insertion_cost(r2, customer, i, problem) - r2.distance;

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

    /// Calculate the cost change when removing a customer from a route.
    fn calculate_removal_cost(&self, route: &Route, pos: usize, problem: &Problem) -> f64 {
        let customers = &route.customers;
        let n = customers.len();

        if n <= 1 {
            // Removing the only customer makes the route empty
            return -route.distance;
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

        // Add new connection
        let new_distance = problem.get_distance(prev_idx, next_idx);

        // The delta is the new distance minus the old distance
        new_distance - old_distance
    }

    /// Calculate the cost of inserting a customer into a route at a specific position.
    fn calculate_insertion_cost(
        &self,
        route: &Route,
        customer: usize,
        pos: usize,
        problem: &Problem,
    ) -> f64 {
        let customers = &route.customers;
        let n = customers.len();

        if n == 0 {
            // Inserting into an empty route
            return problem.get_distance(problem.depot_index, customer) * 2.0;
        }

        // Calculate the change in distance
        let prev_idx = if pos > 0 {
            customers[pos - 1]
        } else {
            problem.depot_index
        };
        let next_idx = if pos < n {
            customers[pos]
        } else {
            problem.depot_index
        };

        // Remove current connection
        let old_distance = problem.get_distance(prev_idx, next_idx);

        // Add new connections
        let new_distance =
            problem.get_distance(prev_idx, customer) + problem.get_distance(customer, next_idx);

        // The new total route distance
        route.distance - old_distance + new_distance
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

    /// Implement the Swap neighborhood.
    fn swap_neighborhood(
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
            let r1 = &solution.routes[r1_idx];

            if r1.is_empty() {
                continue;
            }

            // Try to swap each customer in r1
            let customers = r1.customers.len();
            let mut customer_indices: Vec<usize> = (0..customers).collect();
            customer_indices.shuffle(&mut rng);

            for &c1_pos in &customer_indices {
                let customer1 = r1.customers[c1_pos];

                // Only consider close customers based on granularity
                let neighbors = self.get_neighbors(customer1, problem);

                for &neighbor in &neighbors {
                    // Find which route contains this neighbor
                    let r2_idx = self.find_route_for_customer(solution, neighbor);

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

    /// Implement the 2-Opt neighborhood for intra-route improvements.
    fn two_opt_neighborhood(
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
            let route = &solution.routes[r_idx];

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

    /// Implement the 2-Opt* neighborhood for inter-route improvements.
    fn two_opt_star_neighborhood(
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
            let r1 = &solution.routes[r1_idx];

            if r1.is_empty() {
                continue;
            }

            for r2_pos in r1_pos + 1..route_indices.len() {
                let r2_idx = route_indices[r2_pos];
                let r2 = &solution.routes[r2_idx];

                if r2.is_empty() {
                    continue;
                }

                // Try cutting each route at different positions
                for i in 0..r1.customers.len() {
                    let customer1 = r1.customers[i];

                    // Get neighbors based on granularity
                    let neighbors = self.get_neighbors(customer1, problem);

                    for &neighbor in &neighbors {
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

    /// Implement the SWAP* neighborhood.
    fn swap_star_neighborhood(
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
            let r1 = &solution.routes[r1_idx];

            if r1.is_empty() {
                continue;
            }

            let r1_info = &self.route_sectors[r1_idx];

            for r2_idx in 0..solution.routes.len() {
                if r1_idx == r2_idx {
                    continue;
                }

                let r2 = &solution.routes[r2_idx];

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
            let mut max_angle = 0.0;

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
            let cost = self.calculate_insertion_cost(route, customer, i, problem);
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
        let r1_remove_delta = self.calculate_removal_cost(r1, pos1, problem);
        let r2_remove_delta = self.calculate_removal_cost(r2, pos2, problem);

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
                let r1_temp = self.create_temp_route(r1, pos1, customer2, r1_insert_pos);
                let r1_delta = r1_temp.distance - r1.distance;

                // Calculate r2 delta (remove customer2, insert customer1)
                let r2_insert_pos = if insert_pos2 > pos2 {
                    insert_pos2 - 1
                } else {
                    insert_pos2
                };
                let r2_temp = self.create_temp_route(r2, pos2, customer1, r2_insert_pos);
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

    /// Create a temporary route for evaluation purposes.
    fn create_temp_route(
        &self,
        route: &Route,
        remove_pos: usize,
        insert_customer: usize,
        insert_pos: usize,
    ) -> Route {
        let mut temp_route = Route::new();
        temp_route.distance = 0.0;
        temp_route.load = 0.0;

        // Copy the route without the customer at remove_pos
        for (i, &customer) in route.customers.iter().enumerate() {
            if i != remove_pos {
                temp_route.customers.push(customer);
            }
        }

        // Insert the new customer
        temp_route.customers.insert(insert_pos, insert_customer);

        temp_route
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
