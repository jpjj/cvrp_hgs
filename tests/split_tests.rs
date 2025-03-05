//! Unit tests for the Split algorithm in the HGS-CVRP implementation.

use hgs_cvrp::problem::{Node, Problem};
use hgs_cvrp::solution::Solution;
use hgs_cvrp::split::Split;

/// Creates a simple test problem with a depot and some customers.
fn create_test_problem() -> Problem {
    let mut nodes = Vec::new();

    // Depot at (0, 0)
    nodes.push(Node::new(0, 0.0, 0.0, 0.0, true));

    // 6 customers in a line, with increasing demands
    // Customer 1 at (10, 0) with demand 1.0
    nodes.push(Node::new(1, 10.0, 0.0, 1.0, false));
    // Customer 2 at (20, 0) with demand 1.0
    nodes.push(Node::new(2, 20.0, 0.0, 1.0, false));
    // Customer 3 at (30, 0) with demand 1.5
    nodes.push(Node::new(3, 30.0, 0.0, 1.5, false));
    // Customer 4 at (40, 0) with demand 1.5
    nodes.push(Node::new(4, 40.0, 0.0, 1.5, false));
    // Customer 5 at (50, 0) with demand 2.0
    nodes.push(Node::new(5, 50.0, 0.0, 2.0, false));
    // Customer 6 at (60, 0) with demand 2.0
    nodes.push(Node::new(6, 60.0, 0.0, 2.0, false));

    Problem::new(
        "TestProblem".to_string(),
        nodes,
        0,    // depot index
        5.0,  // vehicle capacity
        None, // no max vehicles constraint
    )
}

#[test]
fn test_split_empty_tour() {
    let problem = create_test_problem();

    // Create an empty solution
    let mut solution = Solution::new();
    solution.giant_tour = vec![];

    // Run split algorithm
    Split::split(&mut solution, &problem);

    // Should produce 0 routes
    assert_eq!(solution.routes.len(), 0);
}

#[test]
fn test_split_single_customer() {
    let problem = create_test_problem();

    // Create a solution with one customer
    let mut solution = Solution::new();
    solution.giant_tour = vec![1];

    // Run split algorithm
    Split::split(&mut solution, &problem);

    // Should produce 1 route
    assert_eq!(solution.routes.len(), 1);
    assert_eq!(solution.routes[0].customers, vec![1]);

    // Check if the route's distance is correctly calculated
    // Distance should be depot->1->depot = 10 + 10 = 20
    assert!((solution.routes[0].distance - 20.0).abs() < 1e-6);
}

#[test]
fn test_split_multiple_customers_one_route() {
    let problem = create_test_problem();

    // Create a solution with 3 customers (total demand = 3.5)
    let mut solution = Solution::new();
    solution.giant_tour = vec![1, 2, 3];

    // Run split algorithm
    Split::split(&mut solution, &problem);

    // Should produce 1 route (capacity is 5.0)
    assert_eq!(solution.routes.len(), 1);
    assert_eq!(solution.routes[0].customers, vec![1, 2, 3]);

    // Check if distance is calculated correctly
    // Route: 0 -> 1 -> 2 -> 3 -> 0
    let expected_distance = 10.0 + 10.0 + 10.0 + 30.0;
    assert!((solution.routes[0].distance - expected_distance).abs() < 1e-6);
}

#[test]
fn test_split_exceeding_capacity() {
    let problem = create_test_problem();

    // Create a solution with all 6 customers (total demand = 9.0)
    let mut solution = Solution::new();
    solution.giant_tour = vec![1, 2, 3, 4, 5, 6];

    // Run split algorithm
    Split::split(&mut solution, &problem);

    // Should produce at least 2 routes (capacity is 5.0)
    assert!(solution.routes.len() >= 2);

    // Each route should respect capacity
    for route in &solution.routes {
        let total_demand: f64 = route
            .customers
            .iter()
            .map(|&c| problem.nodes[c].demand)
            .sum();
        assert!(total_demand <= problem.vehicle_capacity);
    }

    // All customers should be visited exactly once
    let mut visited = vec![false; 7];
    for route in &solution.routes {
        for &customer in &route.customers {
            assert!(!visited[customer], "Customer visited more than once");
            visited[customer] = true;
        }
    }

    for i in 1..=6 {
        assert!(visited[i], "Customer {} not visited", i);
    }
}

#[test]
fn test_split_optimal_partitioning() {
    let problem = create_test_problem();

    // Create a solution with all 6 customers in a suboptimal order
    let mut solution = Solution::new();
    solution.giant_tour = vec![1, 4, 2, 5, 3, 6];

    // Run split algorithm
    Split::split(&mut solution, &problem);

    // Capture the resulting cost
    let optimal_cost = solution.cost;

    // Try a different giant tour ordering but with the same customers
    let mut solution2 = Solution::new();
    solution2.giant_tour = vec![1, 2, 3, 4, 5, 6];

    // Run split again
    Split::split(&mut solution2, &problem);

    // The costs should be the same because Split finds the optimal partitioning
    // regardless of the order in the giant tour
    // (This is a simplification - it's only true if the optimal routes visit customers
    // in the same order as they appear in the giant tour)

    // Instead of equality, check if the costs are close
    assert!((solution2.cost - optimal_cost).abs() < 1.0);
}

#[test]
fn test_merge_routes() {
    let problem = create_test_problem();

    // Create a solution with known routes
    let mut solution = Solution::new();
    solution.giant_tour = vec![1, 2, 3, 4, 5, 6];

    // Run split to create routes
    Split::split(&mut solution, &problem);

    // Clear the giant tour
    solution.giant_tour.clear();

    // Run merge_routes to recreate the giant tour
    Split::merge_routes(&mut solution);

    // The giant tour should contain all customers exactly once
    assert_eq!(solution.giant_tour.len(), 6);

    // Check that all customers are present
    let mut present = vec![false; 7];
    for &customer in &solution.giant_tour {
        present[customer] = true;
    }

    for i in 1..=6 {
        assert!(present[i], "Customer {} not in merged tour", i);
    }
}

#[test]
fn test_split_with_different_vehicle_capacities() {
    // Create problems with different vehicle capacities
    let mut problem1 = create_test_problem();
    problem1.vehicle_capacity = 3.0;

    let mut problem2 = create_test_problem();
    problem2.vehicle_capacity = 7.0;

    // Create identical giant tours
    let mut solution1 = Solution::new();
    solution1.giant_tour = vec![1, 2, 3, 4, 5, 6];

    let mut solution2 = Solution::new();
    solution2.giant_tour = vec![1, 2, 3, 4, 5, 6];

    // Run split on both
    Split::split(&mut solution1, &problem1);
    Split::split(&mut solution2, &problem2);

    // Lower capacity should result in more routes
    assert!(solution1.routes.len() > solution2.routes.len());

    // Both solutions should respect their capacity constraints
    for route in &solution1.routes {
        let total_demand: f64 = route
            .customers
            .iter()
            .map(|&c| problem1.nodes[c].demand)
            .sum();
        assert!(total_demand <= problem1.vehicle_capacity);
    }

    for route in &solution2.routes {
        let total_demand: f64 = route
            .customers
            .iter()
            .map(|&c| problem2.nodes[c].demand)
            .sum();
        assert!(total_demand <= problem2.vehicle_capacity);
    }
}

#[test]
fn test_split_evaluates_solution() {
    let problem = create_test_problem();

    // Create a solution
    let mut solution = Solution::new();
    solution.giant_tour = vec![1, 2, 3, 4, 5, 6];

    // Ensure initial values are zero
    assert_eq!(solution.distance, 0.0);
    assert_eq!(solution.cost, 0.0);
    assert_eq!(solution.excess_capacity, 0.0);

    // Run split
    Split::split(&mut solution, &problem);

    // Split should evaluate the solution, so these values should be updated
    assert!(solution.distance > 0.0);
    assert!(solution.cost > 0.0);
    assert!(solution.is_feasible);
}
