//! Split algorithm implementation to convert a giant tour into routes.

use crate::problem::Problem;
use crate::solution::{Route, Solution};

use std::f64;

/// Implements the Split algorithm to optimally partition a giant tour.
pub struct Split;

impl Split {
    /// Split a giant tour into routes.
    /// This is the linear-time split algorithm from Vidal (2016).
    pub fn split(solution: &mut Solution, problem: &Problem) {
        let giant_tour = &solution.giant_tour;

        if giant_tour.is_empty() {
            solution.routes.clear();
            return;
        }

        let n = giant_tour.len();

        // Auxiliary data structures
        let mut potential = vec![f64::INFINITY; n + 1];
        let mut pred = vec![0; n + 1];

        potential[0] = 0.0;

        // Information about the potential route
        let mut load = 0.0;
        let mut cum_load = vec![0.0; n + 1];

        for i in 0..n {
            load += problem.nodes[giant_tour[i]].demand;
            cum_load[i + 1] = load;
        }

        // DP state
        let mut load_i = 0.0;
        let mut j = 0;

        // For each client
        for i in 0..n {
            // For each potential route (i,j)
            while j < n {
                let route_load = cum_load[j + 1] - cum_load[i];

                if route_load > problem.vehicle_capacity {
                    break;
                }

                // Calculate distance of route i -> j
                let mut route_distance = 0.0;

                // Depot to first
                route_distance += problem.get_distance(problem.depot_index, giant_tour[i]);

                // All successive nodes
                for k in i..j {
                    route_distance += problem.get_distance(giant_tour[k], giant_tour[k + 1]);
                }

                // Last to depot
                route_distance += problem.get_distance(giant_tour[j], problem.depot_index);

                // Calculate new potential
                let new_potential = potential[i] + route_distance;

                if new_potential < potential[j + 1] {
                    potential[j + 1] = new_potential;
                    pred[j + 1] = i;
                }

                j += 1;
            }

            // Refine the DP by allowing more states
            load_i += problem.nodes[giant_tour[i]].demand;

            if j < n && (load_i > problem.vehicle_capacity || i == j) {
                j += 1;
            }
        }

        // Reconstruct the solution
        solution.routes.clear();
        let mut j = n;

        while j > 0 {
            let i = pred[j];

            // Create a new route from i to j-1
            let mut route = Route::new();
            for k in i..j {
                route.customers.push(giant_tour[k]);
            }

            // Calculate route metrics
            route.calculate_load(problem);
            route.calculate_distance(problem);

            // Add route to solution
            solution.routes.push(route);

            j = i;
        }

        // Reverse the routes to get them in the correct order
        solution.routes.reverse();

        // Evaluate the full solution
        solution.evaluate(problem, 1.0); // Default penalty of 1.0, will be adjusted later
    }

    /// Generate a giant tour from a solution's routes.
    pub fn merge_routes(solution: &mut Solution) {
        solution.giant_tour.clear();

        for route in &solution.routes {
            for &customer in &route.customers {
                solution.giant_tour.push(customer);
            }
        }
    }
}
