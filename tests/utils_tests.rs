//! Unit tests for utility functions in the HGS-CVRP algorithm.

use hgs_cvrp::local_search::utils::*;
use hgs_cvrp::problem::{Node, Problem};
use hgs_cvrp::solution::{Route, Solution};
use hgs_cvrp::utils as general_utils;
use std::time::Duration;

/// Creates a simple test problem with a depot and some customers.
fn create_test_problem() -> Problem {
    let mut nodes = Vec::new();

    // Depot at (0, 0)
    nodes.push(Node::new(0, 0.0, 0.0, 0.0, true));

    // 5 customers in a grid
    nodes.push(Node::new(1, 10.0, 0.0, 1.0, false));
    nodes.push(Node::new(2, 0.0, 10.0, 1.0, false));
    nodes.push(Node::new(3, 10.0, 10.0, 1.0, false));
    nodes.push(Node::new(4, 20.0, 0.0, 1.5, false));
    nodes.push(Node::new(5, 20.0, 10.0, 2.0, false));

    Problem::new(
        "TestProblem".to_string(),
        nodes,
        0,    // depot index
        5.0,  // vehicle capacity
        None, // no max vehicles constraint
    )
}

#[test]
fn test_get_neighbors() {
    let problem = create_test_problem();

    // Test with different granularity values
    let neighbors_g1 = get_neighbors(1, &problem, 1);
    assert_eq!(neighbors_g1.len(), 1);

    let neighbors_g3 = get_neighbors(1, &problem, 3);
    assert_eq!(neighbors_g3.len(), 3);

    // Test with granularity larger than available customers
    let neighbors_g10 = get_neighbors(1, &problem, 10);
    assert_eq!(neighbors_g10.len(), 4); // All other customers

    // Check that depot is not included in neighbors
    for &neighbor in &neighbors_g10 {
        assert_ne!(neighbor, 0);
    }

    // Check that self is not included in neighbors
    for &neighbor in &neighbors_g10 {
        assert_ne!(neighbor, 1);
    }
}

#[test]
fn test_find_route_for_customer() {
    let problem = create_test_problem();

    // Create a solution with two routes
    let mut solution = Solution::new();

    let mut route1 = Route::new();
    route1.customers = vec![1, 2];

    let mut route2 = Route::new();
    route2.customers = vec![3, 4, 5];

    solution.routes = vec![route1, route2];

    // Test finding customers
    assert_eq!(find_route_for_customer(&solution, 1), Some(0));
    assert_eq!(find_route_for_customer(&solution, 3), Some(1));

    // Test customer not in solution
    assert_eq!(find_route_for_customer(&solution, 6), None);

    // Test depot
    assert_eq!(find_route_for_customer(&solution, 0), None);
}

#[test]
fn test_calculate_insertion_cost() {
    let problem = create_test_problem();

    // Create a route: Depot -> 1 -> 2 -> Depot
    let mut route = Route::new();
    route.customers = vec![1, 2];
    route.calculate_distance(&problem);
    let original_distance = route.distance;

    // Calculate insertion costs for customer 3
    let cost_at_0 = calculate_insertion_cost(&route, 3, 0, &problem);
    let cost_at_1 = calculate_insertion_cost(&route, 3, 1, &problem);
    let cost_at_2 = calculate_insertion_cost(&route, 3, 2, &problem);

    // Verify with actual insertions
    let mut route0 = route.clone();
    route0.customers.insert(0, 3);
    route0.calculate_distance(&problem);

    let mut route1 = route.clone();
    route1.customers.insert(1, 3);
    route1.calculate_distance(&problem);

    let mut route2 = route.clone();
    route2.customers.insert(2, 3);
    route2.calculate_distance(&problem);

    assert!((cost_at_0 - route0.distance).abs() < 1e-6);
    assert!((cost_at_1 - route1.distance).abs() < 1e-6);
    assert!((cost_at_2 - route2.distance).abs() < 1e-6);
}

#[test]
fn test_calculate_removal_cost() {
    let problem = create_test_problem();

    // Create a route: Depot -> 1 -> 2 -> 3 -> Depot
    let mut route = Route::new();
    route.customers = vec![1, 2, 3];
    route.calculate_distance(&problem);
    let original_distance = route.distance;

    // Calculate removal costs
    let delta0 = calculate_removal_cost(&route, 0, &problem);
    let delta1 = calculate_removal_cost(&route, 1, &problem);
    let delta2 = calculate_removal_cost(&route, 2, &problem);

    // Verify with actual removals
    let mut route0 = route.clone();
    route0.customers.remove(0);
    route0.calculate_distance(&problem);

    let mut route1 = route.clone();
    route1.customers.remove(1);
    route1.calculate_distance(&problem);

    let mut route2 = route.clone();
    route2.customers.remove(2);
    route2.calculate_distance(&problem);

    // Delta should match the actual distance change
    assert!((route0.distance - original_distance - delta0).abs() < 1e-6);
    assert!((route1.distance - original_distance - delta1).abs() < 1e-6);
    assert!((route2.distance - original_distance - delta2).abs() < 1e-6);
}

#[test]
fn test_create_temp_route() {
    // Create a route: Depot -> 1 -> 2 -> 3 -> Depot
    let mut route = Route::new();
    route.customers = vec![1, 2, 3];

    // Test removing customer 1 and inserting customer 4 at position 0
    let temp_route = create_temp_route(&route, 1, 4, 0);

    // The temp route should have customer 4 at position 0, then customer 3
    assert_eq!(temp_route.customers, vec![4, 1, 3]);

    // Test removing customer 0 and inserting customer 5 at the end
    let temp_route2 = create_temp_route(&route, 0, 5, 2);

    // The temp route should have customers 2, 3, 5
    assert_eq!(temp_route2.customers, vec![2, 3, 5]);
}

#[test]
fn test_general_utils_format_duration() {
    // Test some sample durations
    let duration1 = Duration::from_secs(65);
    assert_eq!(general_utils::format_duration(duration1), "0h 01m 05s");

    let duration2 = Duration::from_secs(3600 + 120 + 5);
    assert_eq!(general_utils::format_duration(duration2), "1h 02m 05s");

    let duration3 = Duration::from_secs(7200 + 3600 + 900 + 30);
    assert_eq!(general_utils::format_duration(duration3), "3h 15m 30s");
}

#[test]
fn test_route_info() {
    // Test the RouteInfo struct for polar sectors
    let route_info1 = RouteInfo {
        route_index: 0,
        polar_min: 0.0,
        polar_max: std::f64::consts::PI / 2.0, // 90 degrees
    };

    let route_info2 = RouteInfo {
        route_index: 1,
        polar_min: std::f64::consts::PI / 4.0, // 45 degrees
        polar_max: 3.0 * std::f64::consts::PI / 4.0, // 135 degrees
    };

    let route_info3 = RouteInfo {
        route_index: 2,
        polar_min: std::f64::consts::PI,             // 180 degrees
        polar_max: 3.0 * std::f64::consts::PI / 2.0, // 270 degrees
    };

    // Test that non-overlapping sectors don't intersect
    assert!(!sectors_intersect(&route_info1, &route_info3));

    // Test that overlapping sectors do intersect
    assert!(sectors_intersect(&route_info1, &route_info2));
    assert!(sectors_intersect(&route_info2, &route_info3));
}

// Helper function to test sector intersection logic
fn sectors_intersect(s1: &RouteInfo, s2: &RouteInfo) -> bool {
    // Special case: empty routes
    if s1.polar_min == s1.polar_max || s2.polar_min == s2.polar_max {
        return false;
    }

    // Check for intersection
    !(s1.polar_max < s2.polar_min || s2.polar_max < s1.polar_min)
}

#[test]
fn test_calculate_excess_load() {
    let problem = create_test_problem();

    // Create a solution with two routes
    let mut solution = Solution::new();

    let mut route1 = Route::new();
    route1.customers = vec![1, 2]; // Demand: 1.0 + 1.0 = 2.0
    route1.calculate_load(&problem);

    let mut route2 = Route::new();
    route2.customers = vec![3, 4, 5]; // Demand: 1.0 + 1.5 + 2.0 = 4.5
    route2.calculate_load(&problem);

    solution.routes = vec![route1, route2];

    // Both routes are within capacity (5.0), so excess should be 0
    assert_eq!(
        general_utils::calculate_excess_load(&solution, &problem),
        0.0
    );

    // Add customer to route2 to make it exceed capacity
    let mut overloaded_solution = solution.clone();
    overloaded_solution.routes[1].customers.push(1); // Add 1.0 more demand = 5.5 > 5.0
    overloaded_solution.routes[1].calculate_load(&problem);

    // Should have 0.5 excess
    assert!(
        (general_utils::calculate_excess_load(&overloaded_solution, &problem) - 0.5).abs() < 1e-6
    );
}
