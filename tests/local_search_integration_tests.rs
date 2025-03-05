//! Integration tests for the local search components working together.

use hgs_cvrp::local_search::LocalSearch;
use hgs_cvrp::problem::{Node, Problem};
use hgs_cvrp::solution::{Route, Solution};

/// Creates a more complex test problem with a depot and customers in a grid.
fn create_complex_problem() -> Problem {
    let mut nodes = Vec::new();

    // Depot at (50, 50)
    nodes.push(Node::new(0, 50.0, 50.0, 0.0, true));

    // Generate a grid of customers
    let mut id = 1;
    for x in 0..5 {
        for y in 0..5 {
            // Skip the center where the depot is
            if x == 2 && y == 2 {
                continue;
            }
            let x_coord = x as f64 * 20.0 + 10.0;
            let y_coord = y as f64 * 20.0 + 10.0;
            let demand = 1.0 + (x + y) as f64 * 0.1;
            nodes.push(Node::new(id, x_coord, y_coord, demand, false));
            id += 1;
        }
    }

    Problem::new(
        "ComplexTestProblem".to_string(),
        nodes,
        0,    // depot index
        10.0, // vehicle capacity
        None, // no max vehicles constraint
    )
}

/// Creates a solution with random routes for testing.
fn create_random_solution(problem: &Problem) -> Solution {
    use rand::{seq::SliceRandom, thread_rng};

    let mut solution = Solution::new();
    let mut rng = thread_rng();

    // Create a random giant tour
    let customer_count = problem.get_customer_count();
    let mut giant_tour: Vec<usize> = (1..=customer_count).collect();
    giant_tour.shuffle(&mut rng);

    // Split into 3 routes
    let route_size = giant_tour.len() / 3;

    for i in 0..3 {
        let start = i * route_size;
        let end = if i == 2 {
            giant_tour.len()
        } else {
            (i + 1) * route_size
        };

        let mut route = Route::new();
        route.customers = giant_tour[start..end].to_vec();
        route.calculate_load(problem);
        route.calculate_distance(problem);

        solution.routes.push(route);
    }

    solution.evaluate(problem, 1.0);
    solution.update_giant_tour();

    solution
}

#[test]
fn test_local_search_move_tracking() {
    let problem = create_complex_problem();
    let mut solution = create_random_solution(&problem);

    // Create local search with timestamp tracking
    let mut local_search = LocalSearch::new(5);

    // Check initial validity for a move
    let customer = 1;
    let move_type = 0; // Relocate
    let route_idx = 1;

    assert!(local_search.is_move_valid(customer, move_type, route_idx));

    // The second check should return false (already tested)
    assert!(!local_search.is_move_valid(customer, move_type, route_idx));

    // After updating the route timestamp, the move should be valid again
    local_search.update_route_timestamp(route_idx);
    assert!(local_search.is_move_valid(customer, move_type, route_idx));
}

#[test]
fn test_local_search_initialization_tracking() {
    let problem = create_complex_problem();
    let mut solution = create_random_solution(&problem);

    // Create local search
    let mut local_search = LocalSearch::new(5);

    // Initialize tracking (normally called by educate)
    local_search.initialize_tracking(&solution);

    // All route timestamps should be 0
    assert_eq!(local_search.route_timestamps.len(), solution.routes.len());
    for &ts in &local_search.route_timestamps {
        assert_eq!(ts, 0);
    }

    // Move count should be 0
    assert_eq!(local_search.move_count, 0);

    // Move timestamps should be empty
    assert!(local_search.move_timestamps.is_empty());
}

#[test]
fn test_local_search_full_improvement() {
    // This test checks if running the full local search can improve a solution

    let problem = create_complex_problem();

    // Create a deliberately suboptimal solution
    let mut solution = Solution::new();

    // Route 1: Depot -> 1 -> 10 -> 15 -> Depot (scattered points, not optimal)
    let mut route1 = Route::new();
    route1.customers = vec![1, 10, 15];
    route1.calculate_load(&problem);
    route1.calculate_distance(&problem);

    // Route 2: Depot -> 2 -> 11 -> 16 -> Depot (scattered points, not optimal)
    let mut route2 = Route::new();
    route2.customers = vec![2, 11, 16];
    route2.calculate_load(&problem);
    route2.calculate_distance(&problem);

    // Route 3: Depot -> 3 -> 12 -> 17 -> Depot (scattered points, not optimal)
    let mut route3 = Route::new();
    route3.customers = vec![3, 12, 17];
    route3.calculate_load(&problem);
    route3.calculate_distance(&problem);

    solution.routes = vec![route1, route2, route3];
    solution.evaluate(&problem, 1.0);

    let initial_cost = solution.cost;

    // Run local search
    let mut local_search = LocalSearch::new(10);
    local_search.educate(&mut solution, &problem, 1.0);

    // The solution should improve
    assert!(solution.cost < initial_cost);

    // Check that the solution is still valid
    assert!(solution.is_feasible);

    // Each route should have consistent distance and load
    for route in &solution.routes {
        assert!(route.distance > 0.0);
        assert!(route.load > 0.0);
    }
}

#[test]
fn test_local_search_multiple_iterations() {
    // This test checks if running multiple iterations of local search
    // eventually reaches a local optimum

    let problem = create_complex_problem();
    let mut solution = create_random_solution(&problem);

    let mut local_search = LocalSearch::new(10);

    // First run
    local_search.educate(&mut solution, &problem, 1.0);
    let first_cost = solution.cost;

    // Second run should not improve much if at all
    local_search.educate(&mut solution, &problem, 1.0);
    let second_cost = solution.cost;

    // The second run should not improve by more than a small amount
    // (allowing for minor numerical differences)
    assert!((second_cost - first_cost).abs() < 1e-6);
}

#[test]
fn test_local_search_capacity_penalties() {
    let problem = create_complex_problem();

    // Create an overloaded solution
    let mut solution = Solution::new();
    let mut route = Route::new();

    // Put 15 customers in one route (will exceed capacity)
    route.customers = (1..=15).collect();
    route.calculate_load(&problem);
    route.calculate_distance(&problem);
    solution.routes = vec![route];

    // Evaluate with a low penalty first
    solution.evaluate(&problem, 0.1);
    assert!(!solution.is_feasible);
    let low_penalty_cost = solution.cost;

    // Create a copy to evaluate with high penalty
    let mut high_penalty_solution = solution.clone();
    high_penalty_solution.evaluate(&problem, 10.0);
    let high_penalty_cost = high_penalty_solution.cost;

    // Higher penalty should result in higher cost
    assert!(high_penalty_cost > low_penalty_cost);

    // Now run local search with high penalty
    let mut local_search = LocalSearch::new(10);
    local_search.educate(&mut high_penalty_solution, &problem, 10.0);

    // Local search should try to reduce capacity violations
    assert!(
        high_penalty_solution.excess_capacity < solution.excess_capacity
            || high_penalty_solution.is_feasible
    );
}

#[test]
fn test_repair_function() {
    let problem = create_complex_problem();

    // Create an overloaded solution
    let mut solution = Solution::new();
    let mut route = Route::new();

    // Put 15 customers in one route (will exceed capacity)
    route.customers = (1..=15).collect();
    route.calculate_load(&problem);
    route.calculate_distance(&problem);
    solution.routes = vec![route];
    solution.evaluate(&problem, 1.0);

    assert!(!solution.is_feasible);

    // Try to repair
    let mut local_search = LocalSearch::new(10);
    local_search.repair(&mut solution, &problem);

    // After repair, the solution should have less capacity violation
    // or become feasible
    assert!(solution.is_feasible || solution.excess_capacity < 5.0);
}

#[test]
fn test_preprocess_neighbors() {
    let problem = create_complex_problem();
    let mut local_search = LocalSearch::new(5);

    // Initially, the neighbor map should be empty
    assert!(local_search.customer_neighbors.is_empty());

    // Call preprocess_neighbors
    local_search.preprocess_neighbors(&problem);

    // Now we should have neighbors for each customer
    assert_eq!(
        local_search.customer_neighbors.len(),
        problem.get_customer_count()
    );

    // Each customer should have at most 5 neighbors (granularity)
    for (_, neighbors) in &local_search.customer_neighbors {
        assert!(neighbors.len() <= 5);
    }
}
