//! Integration tests for the full HGS-CVRP algorithm.

use hgs_cvrp::config::Config;
use hgs_cvrp::problem::{Node, Problem};
use hgs_cvrp::HgsAlgorithm;
use std::time::Duration;

/// Creates a moderate size test problem with a depot and customers.
fn create_moderate_problem() -> Problem {
    let mut nodes = Vec::new();

    // Depot at (50, 50)
    nodes.push(Node::new(0, 50.0, 50.0, 0.0, true));

    // Create 20 customers in a grid pattern
    let mut id = 1;
    for i in 0..4 {
        for j in 0..5 {
            let x = i as f64 * 20.0 + 10.0;
            let y = j as f64 * 20.0 + 10.0;
            let demand = 1.0 + 0.1 * (id as f64 % 3.0);
            nodes.push(Node::new(id, x, y, demand, false));
            id += 1;
        }
    }

    Problem::new(
        "ModerateTestProblem".to_string(),
        nodes,
        0,    // depot index
        10.0, // vehicle capacity
        None, // no max vehicles constraint
    )
}

#[test]
fn test_algorithm_initialization() {
    let problem = create_moderate_problem();
    let config = Config::new().with_min_pop_size(10).with_generation_size(20);

    let mut algorithm = HgsAlgorithm::new(problem, config);
    algorithm.initialize();

    // Population should be initialized
    assert!(algorithm.population.get_pop_size() >= 10);

    // Best solution should be set
    assert!(algorithm.best_solution.is_some());

    // The solution should be feasible
    assert!(algorithm.best_solution.as_ref().unwrap().is_feasible);
}

#[test]
fn test_algorithm_short_run() {
    let problem = create_moderate_problem();
    let config = Config::new()
        .with_min_pop_size(10)
        .with_generation_size(20)
        .with_max_iterations_without_improvement(100)
        .with_time_limit(Duration::from_secs(1));

    let mut algorithm = HgsAlgorithm::new(problem, config);
    let vehicle_capacity = algorithm.problem.vehicle_capacity;
    let number_nodes = algorithm.problem.nodes.len();
    // Run the algorithm
    let solution = algorithm.run();
    // Solution should exist and be feasible
    assert!(solution.is_feasible);
    assert!(solution.distance > 0.0);

    // Each route should respect capacity
    for route in &solution.routes {
        assert!(route.load <= vehicle_capacity);
    }

    // All customers should be visited exactly once
    let mut visited = vec![false; number_nodes];
    for route in &solution.routes {
        for &customer in &route.customers {
            assert!(!visited[customer], "Customer visited more than once");
            visited[customer] = true;
        }
    }

    for i in 1..algorithm.problem.nodes.len() {
        assert!(visited[i], "Customer {} not visited", i);
    }
}

#[test]
fn test_algorithm_termination_time_limit() {
    let problem = create_moderate_problem();
    let time_limit = Duration::from_millis(200);

    let config = Config::new()
        .with_min_pop_size(10)
        .with_generation_size(20)
        .with_max_iterations_without_improvement(100000) // Very high to ensure time is the limiting factor
        .with_time_limit(time_limit);

    let mut algorithm = HgsAlgorithm::new(problem, config);

    // Run the algorithm
    algorithm.run();

    // Algorithm should terminate due to time limit
    assert!(algorithm.run_time >= time_limit);
}

#[test]
fn test_algorithm_termination_iterations() {
    let problem = create_moderate_problem();
    let max_iterations = 50;

    let config = Config::new()
        .with_min_pop_size(10)
        .with_generation_size(20)
        .with_max_iterations_without_improvement(max_iterations)
        .with_time_limit(Duration::MAX); // No time limit

    let mut algorithm = HgsAlgorithm::new(problem, config);

    // Run the algorithm
    algorithm.run();

    // Algorithm should terminate due to iteration limit
    assert!(algorithm.iterations_without_improvement >= max_iterations);
}

#[test]
fn test_algorithm_improvement() {
    let problem = create_moderate_problem();

    let config = Config::new()
        .with_min_pop_size(5)
        .with_generation_size(10)
        .with_max_iterations_without_improvement(200)
        .with_time_limit(Duration::from_secs(2));

    let mut algorithm = HgsAlgorithm::new(problem, config);
    algorithm.initialize();

    // Get initial solution quality
    let initial_solution = algorithm.best_solution.clone().unwrap();
    let initial_cost = initial_solution.cost;

    // Run the algorithm
    let final_solution = algorithm.run();

    // Final solution should be better than initial solution
    assert!(final_solution.cost < initial_cost);
}

#[test]
fn test_algorithm_multiple_runs_consistency() {
    // This test checks if multiple runs produce reasonably similar results

    let problem = create_moderate_problem();

    let config = Config::new()
        .with_min_pop_size(10)
        .with_generation_size(20)
        .with_max_iterations_without_improvement(200)
        .with_time_limit(Duration::from_secs(2));

    // First run
    let mut algorithm1 = HgsAlgorithm::new(problem.clone(), config.clone());
    let solution1 = algorithm1.run();

    // Second run
    let mut algorithm2 = HgsAlgorithm::new(problem.clone(), config.clone());
    let solution2 = algorithm2.run();

    // The solutions should be similar in quality (within 20%)
    // But not necessarily identical due to randomness
    let ratio = solution1.cost / solution2.cost;
    assert!(ratio > 0.8 && ratio < 1.25);
}

#[test]
fn test_algorithm_with_different_configs() {
    let problem = create_moderate_problem();

    // Config with small population
    let small_config = Config::new()
        .with_min_pop_size(5)
        .with_generation_size(10)
        .with_max_iterations_without_improvement(100)
        .with_time_limit(Duration::from_secs(1));

    // Config with larger population
    let large_config = Config::new()
        .with_min_pop_size(15)
        .with_generation_size(30)
        .with_max_iterations_without_improvement(100)
        .with_time_limit(Duration::from_secs(1));

    // Run with small config
    let mut small_algorithm = HgsAlgorithm::new(problem.clone(), small_config);
    let small_solution = small_algorithm.run();

    // Run with large config
    let mut large_algorithm = HgsAlgorithm::new(problem.clone(), large_config);
    let large_solution = large_algorithm.run();

    // Both solutions should be feasible
    assert!(small_solution.is_feasible);
    assert!(large_solution.is_feasible);

    // Larger population should (potentially) yield better results
    // but this is not guaranteed due to random factors and limited runtime
    // So we merely check that the results are reasonably similar
    let ratio = small_solution.cost / large_solution.cost;
    assert!(ratio > 0.7 && ratio < 1.3);
}
