//! Unit tests for the genetic components of the HGS-CVRP algorithm.

use hgs_cvrp::config::Config;
use hgs_cvrp::genetic::Genetic;
use hgs_cvrp::individual::Individual;
use hgs_cvrp::population::Population;
use hgs_cvrp::problem::{Node, Problem};
use hgs_cvrp::solution::{Route, Solution};

/// Creates a simple test problem with a depot and some customers.
fn create_test_problem() -> Problem {
    let mut nodes = Vec::new();

    // Depot at (0, 0)
    nodes.push(Node::new(0, 0.0, 0.0, 0.0, true));

    // 9 customers in a 3x3 grid
    for i in 0..3 {
        for j in 0..3 {
            let id = i * 3 + j + 1;
            let x = (i as f64 + 1.0) * 10.0;
            let y = (j as f64 + 1.0) * 10.0;
            let demand = 1.0;
            nodes.push(Node::new(id, x, y, demand, false));
        }
    }

    Problem::new(
        "TestProblem".to_string(),
        nodes,
        0,    // depot index
        5.0,  // vehicle capacity
        None, // no max vehicles constraint
    )
}

/// Create a sample solution for testing.
fn create_test_solution() -> Solution {
    let mut solution = Solution::new();
    solution.giant_tour = vec![1, 2, 3, 4, 5, 6, 7, 8, 9];
    solution
}

/// Create a sample individual for testing.
fn create_test_individual() -> Individual {
    let solution = create_test_solution();
    Individual::new(solution)
}

#[test]
fn test_genetic_crossover() {
    let genetic = Genetic;

    // Create two parent solutions with different giant tours
    let mut parent1 = create_test_individual();
    parent1.solution.giant_tour = vec![1, 2, 3, 4, 5, 6, 7, 8, 9];

    let mut parent2 = create_test_individual();
    parent2.solution.giant_tour = vec![9, 8, 7, 6, 5, 4, 3, 2, 1];

    // Perform crossover
    let offspring = genetic.crossover(&parent1, &parent2);

    // Offspring should have a valid giant tour
    assert_eq!(offspring.giant_tour.len(), 9);

    // Check that all customers are present exactly once
    let mut present = vec![false; 10];
    for &customer in &offspring.giant_tour {
        assert!(!present[customer], "Customer present more than once");
        present[customer] = true;
    }

    for i in 1..=9 {
        assert!(present[i], "Customer {} not present", i);
    }

    // Offspring should inherit genetic material from both parents
    let from_parent1 = offspring
        .giant_tour
        .iter()
        .zip(parent1.solution.giant_tour.iter())
        .filter(|&(&a, &b)| a == b)
        .count();

    let from_parent2 = offspring
        .giant_tour
        .iter()
        .zip(parent2.solution.giant_tour.iter())
        .filter(|&(&a, &b)| a == b)
        .count();

    // The offspring should contain genetic material from both parents
    // But since crossover is random, we can't make too strict assertions
    assert!(from_parent1 > 0 || from_parent2 > 0);
}

#[test]
fn test_genetic_mutate() {
    let genetic = Genetic;

    // Create an individual for mutation
    let mut individual = create_test_individual();
    let original_tour = individual.solution.giant_tour.clone();

    // Apply mutation with a high rate to ensure changes
    genetic.mutate(&mut individual, 1.0);

    // The tour should still be valid
    assert_eq!(individual.solution.giant_tour.len(), 9);

    // All customers should still be present
    let mut present = vec![false; 10];
    for &customer in &individual.solution.giant_tour {
        present[customer] = true;
    }

    for i in 1..=9 {
        assert!(present[i], "Customer {} not present after mutation", i);
    }

    // The tour should have changed due to mutation
    assert_ne!(individual.solution.giant_tour, original_tour);
}

#[test]
fn test_individual_calculate_biased_fitness() {
    // Create an individual for testing
    let mut individual = create_test_individual();

    // Set ranks
    individual.rank_feasibility = 3;
    individual.rank_diversity = 5;

    // Calculate biased fitness with different elite proportions
    individual.calculate_biased_fitness(0.0);
    let fitness1 = individual.biased_fitness;

    individual.calculate_biased_fitness(0.5);
    let fitness2 = individual.biased_fitness;

    // With higher elite proportion, diversity should be weighted less
    assert!(fitness2 < fitness1);

    // With elite_proportion = 0, fitness should equal rank_feasibility + rank_diversity
    individual.calculate_biased_fitness(0.0);
    assert_eq!(individual.biased_fitness, 3.0 + 5.0);
}

#[test]
fn test_individual_calculate_diversity_contribution() {
    // Create an individual for testing
    let mut individual = create_test_individual();

    // Set common pairs array
    individual.common_pairs = vec![5, 3, 8, 1, 9];

    // Calculate diversity contribution with different closest counts
    let diversity1 = individual.calculate_diversity_contribution(2);
    let diversity2 = individual.calculate_diversity_contribution(4);

    // With closest_count = 2, should average the 2 highest values: (9+8)/2 = 8.5
    assert_eq!(diversity1, 8.5);

    // With closest_count = 4, should average the 4 highest values: (9+8+5+3)/4 = 6.25
    assert_eq!(diversity2, 6.25);
}

#[test]
fn test_individual_is_clone_of() {
    // Create two individuals with identical tours
    let mut individual1 = create_test_individual();
    individual1.solution.giant_tour = vec![1, 2, 3, 4, 5];

    let mut individual2 = create_test_individual();
    individual2.solution.giant_tour = vec![1, 2, 3, 4, 5];

    // They should be identified as clones
    assert!(individual1.is_clone_of(&individual2));

    // Modify one tour
    individual2.solution.giant_tour = vec![1, 2, 3, 5, 4];

    // They should no longer be clones
    assert!(!individual1.is_clone_of(&individual2));
}

#[test]
fn test_individual_calculate_common_pairs() {
    // Create two individuals with partly similar tours
    let mut individual1 = create_test_individual();
    individual1.solution.giant_tour = vec![1, 2, 3, 4, 5];

    let mut individual2 = create_test_individual();
    individual2.solution.giant_tour = vec![1, 2, 6, 4, 5];

    // Calculate common pairs
    let common_pairs = individual1.calculate_common_pairs(&individual2);

    // Expected common pairs: (1,2) and (4,5) = 2
    assert_eq!(common_pairs, 2);
}

#[test]
fn test_population_initialization() {
    let problem = create_test_problem();
    let config = Config::new().with_min_pop_size(5).with_generation_size(10);

    let mut population = Population::new(&config);
    population.initialize(&problem, &config);

    // Should have initialized at least min_pop_size individuals
    assert!(population.get_pop_size() >= config.min_pop_size);

    // All individuals should have valid solutions
    for individual in &population.feasible_individuals {
        assert_eq!(individual.solution.giant_tour.len(), 9);
    }

    for individual in &population.infeasible_individuals {
        assert_eq!(individual.solution.giant_tour.len(), 9);
    }
}

#[test]
fn test_population_insert_individual() {
    let config = Config::new();
    let mut population = Population::new(&config);

    // Create and insert a feasible individual
    let mut feasible = create_test_individual();
    feasible.solution.is_feasible = true;
    population.insert_individual(feasible);

    // Create and insert an infeasible individual
    let mut infeasible = create_test_individual();
    infeasible.solution.is_feasible = false;
    population.insert_individual(infeasible);

    // Each should go to the appropriate subpopulation
    assert_eq!(population.feasible_individuals.len(), 1);
    assert_eq!(population.infeasible_individuals.len(), 1);
}

#[test]
fn test_population_update_ranks() {
    let config = Config::new().with_n_closest(2);
    let mut population = Population::new(&config);

    // Add several individuals with different costs
    for i in 0..5 {
        let mut individual = create_test_individual();
        individual.solution.cost = (i as f64) * 10.0;
        individual.solution.is_feasible = true;
        population.insert_individual(individual);
    }

    // Update ranks
    population.update_ranks();

    // Check that ranks are assigned correctly
    for i in 0..5 {
        assert_eq!(population.feasible_individuals[i].rank_feasibility, i);
        assert!(population.feasible_individuals[i].rank_diversity >= 0);
        assert!(population.feasible_individuals[i].biased_fitness > 0.0);
    }
}

#[test]
fn test_population_select_parents() {
    let config = Config::new();
    let mut population = Population::new(&config);

    // Add several individuals
    for i in 0..10 {
        let mut individual = create_test_individual();
        individual.solution.cost = (i as f64) * 10.0;
        individual.solution.is_feasible = true;
        population.insert_individual(individual);
    }

    // Update ranks for proper selection
    population.update_ranks();

    // Select parents
    let (parent1, parent2) = population.select_parents();

    // Parents should be different individuals
    assert!(!std::ptr::eq(parent1, parent2));

    // Both should be from the feasible subpopulation
    assert!(parent1.solution.is_feasible);
    assert!(parent2.solution.is_feasible);
}

#[test]
fn test_population_select_survivors() {
    let config = Config::new().with_min_pop_size(5).with_generation_size(10);

    let mut population = Population::new(&config);

    // Add many individuals to exceed min_pop_size
    for i in 0..15 {
        let mut individual = create_test_individual();
        individual.solution.cost = (i as f64) * 10.0;
        individual.solution.is_feasible = true;
        population.insert_individual(individual);
    }

    // Update ranks for proper selection
    population.update_ranks();

    // Initial population size should be 15
    assert_eq!(population.feasible_individuals.len(), 15);

    // Select survivors
    population.select_survivors();

    // Population should be reduced to min_pop_size
    assert_eq!(population.feasible_individuals.len(), config.min_pop_size);

    // The best individuals (lowest cost) should be kept
    for i in 0..config.min_pop_size {
        assert!(population.feasible_individuals[i].solution.cost <= 40.0);
    }
}

#[test]
fn test_population_adjust_penalties() {
    let config = Config::new().with_target_feasible_ratio(0.5);
    let mut population = Population::new(&config);

    // Add 4 feasible and 6 infeasible individuals
    for i in 0..4 {
        let mut individual = create_test_individual();
        individual.solution.is_feasible = true;
        population.insert_individual(individual);
    }

    for i in 0..6 {
        let mut individual = create_test_individual();
        individual.solution.is_feasible = false;
        population.insert_individual(individual);
    }

    // Initial penalty
    let initial_penalty = population.capacity_penalty;

    // Current ratio is 0.4, which is less than target 0.5
    // Adjust penalties - should increase penalty
    population.adjust_penalties();

    // Penalty should increase
    assert!(population.capacity_penalty > initial_penalty);

    // Add more feasible individuals to change the ratio
    for i in 0..7 {
        let mut individual = create_test_individual();
        individual.solution.is_feasible = true;
        population.insert_individual(individual);
    }

    // Now ratio is 11/17 ≈ 0.65, which is more than target 0.5
    // Adjust penalties - should decrease penalty
    let high_penalty = population.capacity_penalty;
    population.adjust_penalties();

    // Penalty should decrease
    assert!(population.capacity_penalty < high_penalty);
}

#[test]
fn test_population_get_best_feasible_solution() {
    let config = Config::new();
    let mut population = Population::new(&config);

    // Add several feasible individuals with different costs
    for i in 0..5 {
        let mut individual = create_test_individual();
        individual.solution.cost = ((5 - i) as f64) * 10.0; // Decreasing costs
        individual.solution.is_feasible = true;
        population.insert_individual(individual);
    }

    // Get best feasible solution
    let best = population.get_best_feasible_solution();

    // Should return the solution with lowest cost (10.0)
    assert!(best.is_some());
    assert_eq!(best.unwrap().cost, 10.0);

    // Add an infeasible solution with even lower cost
    let mut infeasible = create_test_individual();
    infeasible.solution.cost = 5.0;
    infeasible.solution.is_feasible = false;
    population.insert_individual(infeasible);

    // Best feasible should still be the same
    let best = population.get_best_feasible_solution();
    assert_eq!(best.unwrap().cost, 10.0);
}
