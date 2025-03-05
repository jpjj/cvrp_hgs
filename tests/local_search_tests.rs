//! Unit tests for the local search components of the HGS-CVRP algorithm.

use hgs_cvrp::local_search::{utils, LocalSearch};
use hgs_cvrp::problem::{Node, Problem};
use hgs_cvrp::solution::{Route, Solution};

/// Creates a simple test problem with a depot and some customers in a grid.
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

/// Creates a basic solution with two routes for testing.
fn create_test_solution(problem: &Problem) -> Solution {
    let mut solution = Solution::new();

    // Route 1: Depot -> 1 -> 2 -> Depot
    let mut route1 = Route::new();
    route1.customers = vec![1, 2];
    route1.calculate_load(problem);
    route1.calculate_distance(problem);

    // Route 2: Depot -> 3 -> 4 -> 5 -> Depot
    let mut route2 = Route::new();
    route2.customers = vec![3, 4, 5];
    route2.calculate_load(problem);
    route2.calculate_distance(problem);

    solution.routes = vec![route1, route2];
    solution.evaluate(problem, 1.0);
    solution.update_giant_tour();

    solution
}

#[test]
fn test_relocate_neighborhood() {
    let problem = create_test_problem();
    let mut solution = create_test_solution(&problem);

    // Initial solution cost
    let initial_cost = solution.cost;

    // Create local search with small granularity for testing
    let mut local_search = LocalSearch::new(3);

    // Apply relocate neighborhood
    let improved = local_search.relocate_neighborhood(&mut solution, &problem, 1.0);

    // Check if solution improved (cost decreased)
    if improved {
        assert!(solution.cost < initial_cost);
    }

    // Even if no improvement, the solution should be evaluated correctly
    assert!(solution.distance > 0.0);
}

#[test]
fn test_swap_neighborhood() {
    let problem = create_test_problem();
    let mut solution = create_test_solution(&problem);

    // Initial solution cost
    let initial_cost = solution.cost;

    // Create local search with small granularity for testing
    let mut local_search = LocalSearch::new(3);

    // Apply swap neighborhood
    let improved = local_search.swap_neighborhood(&mut solution, &problem, 1.0);

    // Check if solution improved (cost decreased)
    if improved {
        assert!(solution.cost < initial_cost);
    }

    // Even if no improvement, the solution should be evaluated correctly
    assert!(solution.distance > 0.0);
}

#[test]
fn test_two_opt_neighborhood() {
    let problem = create_test_problem();

    // Create a solution with a single route that has crossing edges
    let mut solution = Solution::new();
    let mut route = Route::new();
    // Depot -> 1 -> 3 -> 2 -> 4 -> Depot creates a crossing
    route.customers = vec![1, 3, 2, 4];
    route.calculate_load(&problem);
    route.calculate_distance(&problem);
    solution.routes = vec![route];
    solution.evaluate(&problem, 1.0);

    // Initial solution cost
    let initial_cost = solution.cost;

    // Create local search with small granularity for testing
    let mut local_search = LocalSearch::new(3);

    // Apply 2-opt neighborhood
    let improved = local_search.two_opt_neighborhood(&mut solution, &problem, 1.0);

    // The 2-opt should improve this solution by removing the crossing
    assert!(improved);
    assert!(solution.cost < initial_cost);
}

#[test]
fn test_two_opt_star_neighborhood() {
    let problem = create_test_problem();
    let mut solution = create_test_solution(&problem);

    // Initial solution cost
    let initial_cost = solution.cost;

    // Create local search with small granularity for testing
    let mut local_search = LocalSearch::new(3);

    // Apply 2-opt* neighborhood
    let improved = local_search.two_opt_star_neighborhood(&mut solution, &problem, 1.0);

    // Check if solution improved (cost decreased)
    if improved {
        assert!(solution.cost < initial_cost);
    }

    // Even if no improvement, the solution should be evaluated correctly
    assert!(solution.distance > 0.0);
}

#[test]
fn test_swap_star_neighborhood() {
    let problem = create_test_problem();
    let mut solution = create_test_solution(&problem);

    // Initial solution cost
    let initial_cost = solution.cost;

    // Create local search with small granularity for testing
    let mut local_search = LocalSearch::new(3);

    // Apply SWAP* neighborhood
    let improved = local_search.swap_star_neighborhood(&mut solution, &problem, 1.0);

    // Check if solution improved (cost decreased)
    if improved {
        assert!(solution.cost < initial_cost);
    }

    // Even if no improvement, the solution should be evaluated correctly
    assert!(solution.distance > 0.0);
}

#[test]
fn test_full_educate() {
    let problem = create_test_problem();
    let mut solution = create_test_solution(&problem);

    // Initial solution cost
    let initial_cost = solution.cost;

    // Create local search with small granularity for testing
    let mut local_search = LocalSearch::new(3);

    // Apply full local search educate
    local_search.educate(&mut solution, &problem, 1.0);

    // Local search should either improve or leave the solution unchanged
    assert!(solution.cost <= initial_cost);
}

#[test]
fn test_repair_infeasible_solution() {
    let problem = create_test_problem();

    // Create an infeasible solution (exceeding capacity)
    let mut solution = Solution::new();
    let mut route = Route::new();
    // All customers in one route exceeds capacity (total demand is 6.5)
    route.customers = vec![1, 2, 3, 4, 5];
    route.calculate_load(&problem);
    route.calculate_distance(&problem);
    solution.routes = vec![route];
    solution.evaluate(&problem, 1.0);

    // Solution should be infeasible
    assert!(!solution.is_feasible);

    // Create local search with small granularity for testing
    let mut local_search = LocalSearch::new(3);

    // Try to repair the solution
    local_search.repair(&mut solution, &problem);

    // The repair function should either make the solution feasible
    // or at least reduce the capacity violations
    assert!(solution.excess_capacity < 6.5 - 5.0 || solution.is_feasible);
}

#[test]
fn test_utils_get_neighbors() {
    let problem = create_test_problem();

    // Get neighbors for customer 1 with granularity 3
    let neighbors = utils::get_neighbors(1, &problem, 3);

    // We should get at most 3 neighbors
    assert!(neighbors.len() <= 3);

    // All neighbors should be valid customer indices
    for &neighbor in &neighbors {
        assert!(neighbor > 0 && neighbor < problem.nodes.len());
        assert_ne!(neighbor, 1); // Not self
        assert_ne!(neighbor, 0); // Not depot
    }

    // Neighbors should be ordered by distance from customer 1
    let mut prev_dist = 0.0;
    for &neighbor in &neighbors {
        let dist = problem.get_distance(1, neighbor);
        if prev_dist > 0.0 {
            assert!(dist >= prev_dist);
        }
        prev_dist = dist;
    }
}

#[test]
fn test_utils_find_route_for_customer() {
    let problem = create_test_problem();
    let solution = create_test_solution(&problem);

    // Customer 1 should be in route 0
    let route_idx = utils::find_route_for_customer(&solution, 1);
    assert_eq!(route_idx, Some(0));

    // Customer 5 should be in route 1
    let route_idx = utils::find_route_for_customer(&solution, 5);
    assert_eq!(route_idx, Some(1));

    // Customer 6 doesn't exist, should return None
    let route_idx = utils::find_route_for_customer(&solution, 6);
    assert_eq!(route_idx, None);
}

#[test]
fn test_utils_calculate_insertion_cost() {
    let problem = create_test_problem();

    // Create a simple route: 0 -> 1 -> 2 -> 0
    let mut route = Route::new();
    route.customers = vec![1, 2];
    route.calculate_distance(&problem);

    // Calculate cost of inserting customer 3 at different positions
    let cost_at_0 = utils::calculate_insertion_cost(&route, 3, 0, &problem);
    let cost_at_1 = utils::calculate_insertion_cost(&route, 3, 1, &problem);
    let cost_at_2 = utils::calculate_insertion_cost(&route, 3, 2, &problem);

    // All costs should be non-negative
    assert!(cost_at_0 >= 0.0);
    assert!(cost_at_1 >= 0.0);
    assert!(cost_at_2 >= 0.0);

    // Costs should be different for different positions
    assert!(cost_at_0 != cost_at_1 || cost_at_1 != cost_at_2 || cost_at_0 != cost_at_2);
}

#[test]
fn test_utils_calculate_removal_cost() {
    let problem = create_test_problem();

    // Create a simple route: 0 -> 1 -> 2 -> 0
    let mut route = Route::new();
    route.customers = vec![1, 2];
    route.calculate_distance(&problem);
    let original_distance = route.distance;

    // Calculate cost change when removing each customer
    let delta_0 = utils::calculate_removal_cost(&route, 0, &problem);
    let delta_1 = utils::calculate_removal_cost(&route, 1, &problem);

    // Create routes with each customer removed to verify the delta
    let mut route_without_0 = Route::new();
    route_without_0.customers = vec![2];
    route_without_0.calculate_distance(&problem);

    let mut route_without_1 = Route::new();
    route_without_1.customers = vec![1];
    route_without_1.calculate_distance(&problem);

    // The delta should match the difference in distances
    assert!((route_without_0.distance - original_distance - delta_0).abs() < 1e-6);
    assert!((route_without_1.distance - original_distance - delta_1).abs() < 1e-6);
}
