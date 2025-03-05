//! Unit tests for the Solution and Route structures in the HGS-CVRP implementation.

use hgs_cvrp::problem::{Node, Problem};
use hgs_cvrp::solution::{Route, Solution};

/// Creates a simple test problem with a depot and some customers.
fn create_test_problem() -> Problem {
    let mut nodes = Vec::new();

    // Depot at (0, 0)
    nodes.push(Node::new(0, 0.0, 0.0, 0.0, true));

    // 5 customers in a grid
    // Customer 1 at (10, 0) with demand 1.0
    nodes.push(Node::new(1, 10.0, 0.0, 1.0, false));
    // Customer 2 at (0, 10) with demand 1.0
    nodes.push(Node::new(2, 0.0, 10.0, 1.0, false));
    // Customer 3 at (10, 10) with demand 1.0
    nodes.push(Node::new(3, 10.0, 10.0, 1.0, false));
    // Customer 4 at (20, 0) with demand 1.5
    nodes.push(Node::new(4, 20.0, 0.0, 1.5, false));
    // Customer 5 at (20, 10) with demand 2.0
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
fn test_route_creation() {
    // Test creating an empty route
    let route = Route::new();
    assert!(route.customers.is_empty());
    assert_eq!(route.load, 0.0);
    assert_eq!(route.distance, 0.0);
    assert!(route.modified);

    // Test creating a route with a single customer
    let route = Route::with_customer(1, 1.0, 10.0);
    assert_eq!(route.customers, vec![1]);
    assert_eq!(route.load, 1.0);
    assert_eq!(route.distance, 20.0); // 10.0 * 2 for round trip
    assert!(route.modified);
}

#[test]
fn test_route_calculate_distance() {
    let problem = create_test_problem();

    // Create a route with multiple customers
    let mut route = Route::new();
    route.customers = vec![1, 3, 5];

    // Initially, distance should be 0 and route marked as modified
    assert_eq!(route.distance, 0.0);
    assert!(route.modified);

    // Calculate distance
    route.calculate_distance(&problem);

    // Route should no longer be marked as modified
    assert!(!route.modified);

    // Calculate expected distance
    // Depot -> 1 -> 3 -> 5 -> Depot
    let expected_distance = 10.0 + 10.0 + 10.0 + 20.0;
    assert!((route.distance - expected_distance).abs() < 1e-6);

    // Calling calculate_distance again without modifications should not change the value
    let original_distance = route.distance;
    route.calculate_distance(&problem);
    assert_eq!(route.distance, original_distance);

    // Modify the route and recalculate
    route.customers.push(4);
    route.modified = true;
    route.calculate_distance(&problem);

    // Distance should now be different
    assert!(route.distance != original_distance);
}

#[test]
fn test_route_calculate_load() {
    let problem = create_test_problem();

    // Create a route with multiple customers
    let mut route = Route::new();
    route.customers = vec![1, 3, 5];

    // Calculate load
    route.calculate_load(&problem);

    // Expected load: 1.0 + 1.0 + 2.0 = 4.0
    assert_eq!(route.load, 4.0);

    // Add a customer and recalculate
    route.customers.push(4);
    route.modified = true;
    route.calculate_load(&problem);

    // Expected load: 1.0 + 1.0 + 2.0 + 1.5 = 5.5
    assert_eq!(route.load, 5.5);
}

#[test]
fn test_route_exceeds_capacity() {
    let problem = create_test_problem();

    // Create a route within capacity
    let mut route = Route::new();
    route.customers = vec![1, 2, 3]; // Total demand: 3.0
    route.calculate_load(&problem);

    // Should not exceed capacity of 5.0
    assert!(!route.exceeds_capacity(problem.vehicle_capacity));
    assert_eq!(route.get_excess_load(problem.vehicle_capacity), 0.0);

    // Add more customers to exceed capacity
    route.customers.push(4);
    route.customers.push(5); // Total demand: 6.5
    route.calculate_load(&problem);

    // Should now exceed capacity
    assert!(route.exceeds_capacity(problem.vehicle_capacity));
    assert!((route.get_excess_load(problem.vehicle_capacity) - 1.5).abs() < 1e-6);
}

#[test]
fn test_solution_creation() {
    // Test creating an empty solution
    let solution = Solution::new();
    assert!(solution.routes.is_empty());
    assert_eq!(solution.cost, 0.0);
    assert_eq!(solution.distance, 0.0);
    assert_eq!(solution.excess_capacity, 0.0);
    assert!(solution.is_feasible);
    assert!(solution.giant_tour.is_empty());

    // Test creating a solution from a giant tour
    let problem = create_test_problem();
    let giant_tour = vec![1, 2, 3, 4, 5];
    let solution = Solution::from_giant_tour(giant_tour.clone(), &problem);

    assert!(solution.routes.is_empty()); // Routes are generated later by Split algorithm
    assert_eq!(solution.giant_tour, giant_tour);
}

#[test]
fn test_solution_evaluate() {
    let problem = create_test_problem();

    // Create a solution with two routes
    let mut solution = Solution::new();

    let mut route1 = Route::new();
    route1.customers = vec![1, 2]; // Demand: 2.0

    let mut route2 = Route::new();
    route2.customers = vec![3, 4, 5]; // Demand: 4.5

    solution.routes = vec![route1, route2];

    // Initially, routes have not been evaluated
    assert_eq!(solution.distance, 0.0);
    assert_eq!(solution.cost, 0.0);

    // Evaluate solution
    solution.evaluate(&problem, 1.0);

    // Solution should now have proper values
    assert!(solution.distance > 0.0);
    assert!(solution.cost > 0.0);
    assert!(solution.is_feasible); // Total demand doesn't exceed capacity

    // Create an infeasible solution
    let mut infeasible_solution = Solution::new();

    let mut overloaded_route = Route::new();
    overloaded_route.customers = vec![1, 2, 3, 4, 5]; // Demand: 6.5 > 5.0
    infeasible_solution.routes = vec![overloaded_route];

    // Evaluate with a penalty of 10.0
    infeasible_solution.evaluate(&problem, 10.0);

    // Solution should be infeasible
    assert!(!infeasible_solution.is_feasible);
    assert!(infeasible_solution.excess_capacity > 0.0);

    // Cost should include the penalty: distance + 10.0 * excess
    let expected_cost = infeasible_solution.distance + 10.0 * infeasible_solution.excess_capacity;
    assert!((infeasible_solution.cost - expected_cost).abs() < 1e-6);
}

#[test]
fn test_solution_update_giant_tour() {
    let problem = create_test_problem();

    // Create a solution with two routes
    let mut solution = Solution::new();

    let mut route1 = Route::new();
    route1.customers = vec![1, 2];

    let mut route2 = Route::new();
    route2.customers = vec![3, 4, 5];

    solution.routes = vec![route1, route2];

    // Initially, giant tour is empty
    assert!(solution.giant_tour.is_empty());

    // Update giant tour
    solution.update_giant_tour();

    // Giant tour should now contain all customers in route order
    assert_eq!(solution.giant_tour, vec![1, 2, 3, 4, 5]);
}

#[test]
fn test_solution_get_feasible_cost() {
    let problem = create_test_problem();

    // Create a feasible solution
    let mut feasible_solution = Solution::new();
    let mut route = Route::new();
    route.customers = vec![1, 2, 3]; // Demand: 3.0
    feasible_solution.routes = vec![route];
    feasible_solution.evaluate(&problem, 1.0);

    // Feasible cost should equal the distance
    assert_eq!(
        feasible_solution.get_feasible_cost(),
        feasible_solution.distance
    );

    // Create an infeasible solution
    let mut infeasible_solution = Solution::new();
    let mut overloaded_route = Route::new();
    overloaded_route.customers = vec![1, 2, 3, 4, 5]; // Demand: 6.5 > 5.0
    infeasible_solution.routes = vec![overloaded_route];
    infeasible_solution.evaluate(&problem, 1.0);

    // Infeasible cost should be infinity
    assert_eq!(infeasible_solution.get_feasible_cost(), f64::INFINITY);
}

#[test]
fn test_solution_debug_output() {
    let problem = create_test_problem();

    // Create a simple solution
    let mut solution = Solution::new();
    let mut route = Route::new();
    route.customers = vec![1, 2];
    route.calculate_load(&problem);
    route.calculate_distance(&problem);
    solution.routes = vec![route];
    solution.evaluate(&problem, 1.0);

    // Get debug representation
    let debug_output = format!("{:?}", solution);

    // Debug output should contain relevant information
    assert!(debug_output.contains("Cost:"));
    assert!(debug_output.contains("Distance:"));
    assert!(debug_output.contains("Feasible:"));
    assert!(debug_output.contains("Routes:"));
    assert!(debug_output.contains("Route 0:"));
}
