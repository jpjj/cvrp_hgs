//! Utility functions for local search operations.

use crate::problem::Problem;
use crate::solution::{Route, Solution};
use std::f64;

/// A structure to hold route information for swap* neighborhood.
#[derive(Clone, Copy)]
pub struct RouteInfo {
    pub route_index: usize,
    pub polar_min: f64,
    pub polar_max: f64,
}

/// Generate a list of neighbors for a customer based on granularity.
pub fn get_neighbors(customer: usize, problem: &Problem, granularity: usize) -> Vec<usize> {
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
    let count = std::cmp::min(granularity, distances.len());
    distances.truncate(count);

    distances.into_iter().map(|(idx, _)| idx).collect()
}

/// Find which route contains a specific customer.
pub fn find_route_for_customer(solution: &Solution, customer: usize) -> Option<usize> {
    for (idx, route) in solution.routes.iter().enumerate() {
        if route.customers.contains(&customer) {
            return Some(idx);
        }
    }
    None
}

/// Calculate the cost of inserting a customer into a route at a specific position.
pub fn calculate_insertion_cost(
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

/// Calculate the cost change when removing a customer from a route.
pub fn calculate_removal_cost(route: &Route, pos: usize, problem: &Problem) -> f64 {
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

/// Create a temporary route for evaluation purposes (used in SWAP*).
pub fn create_temp_route(
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
