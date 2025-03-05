//! Basic example of using the HGS-CVRP library.

use hgs_cvrp::config::Config;
use hgs_cvrp::problem::Problem;
use hgs_cvrp::utils::{format_duration, print_solution_visualization, save_solution};
use std::env;
use std::path::Path;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Get instance path from command line or use default
    let args: Vec<String> = env::args().collect();
    let instance_path = if args.len() > 1 {
        &args[1]
    } else {
        "instances/X-n101-k25.vrp"
    };

    // Load problem
    println!("Loading problem from: {}", instance_path);
    let problem = Problem::from_file(instance_path)?;
    println!(
        "Loaded problem: {} with {} customers",
        problem.name,
        problem.get_customer_count()
    );

    // Configure algorithm
    let config = Config::new()
        .with_min_pop_size(25)
        .with_generation_size(40)
        .with_n_elite(4)
        .with_n_closest(5)
        .with_granularity(20)
        .with_target_feasible_ratio(0.2)
        .with_max_iterations_without_improvement(20000)
        .with_time_limit(Duration::from_secs(60));

    // Create and run algorithm
    println!("Initializing algorithm");
    let mut algorithm = hgs_cvrp::HgsAlgorithm::new(problem.clone(), config);

    println!("Starting search (time limit: 60s)");
    let start_time = Instant::now();
    let best_solution = algorithm.run();
    let runtime = start_time.elapsed();

    // Print results
    println!("Search completed in {}", format_duration(runtime));
    println!("Best solution distance: {:.2}", best_solution.distance);
    println!("Is feasible: {}", best_solution.is_feasible);
    println!("Number of routes: {}", best_solution.routes.len());

    // Save solution
    let output_path = format!("{}.sol", problem.name);
    println!("Saving solution to: {}", output_path);
    save_solution(best_solution, &problem, &output_path)?;

    // Visualize solution
    print_solution_visualization(best_solution, &problem);

    Ok(())
}
