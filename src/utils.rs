//! Utility functions and structures for the HGS-CVRP algorithm.

use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::time::Duration;

use crate::problem::Problem;
use crate::solution::Solution;

/// Format a duration as hours, minutes, and seconds.
pub fn format_duration(duration: Duration) -> String {
    let total_seconds = duration.as_secs();
    let hours = total_seconds / 3600;
    let minutes = (total_seconds % 3600) / 60;
    let seconds = total_seconds % 60;

    format!("{}h {:02}m {:02}s", hours, minutes, seconds)
}

/// Save a solution to a file.
pub fn save_solution<P: AsRef<Path>>(
    solution: &Solution,
    problem: &Problem,
    path: P,
) -> std::io::Result<()> {
    let mut file = File::create(path)?;

    writeln!(file, "CVRP Solution for instance: {}", problem.name)?;
    writeln!(file, "Total Distance: {:.2}", solution.distance)?;
    writeln!(file, "Is Feasible: {}", solution.is_feasible)?;
    writeln!(file, "Number of Routes: {}", solution.routes.len())?;
    writeln!(file, "")?;

    for (i, route) in solution.routes.iter().enumerate() {
        write!(file, "Route #{}: ", i + 1)?;

        if route.is_empty() {
            writeln!(file, "Empty")?;
            continue;
        }

        write!(file, "0")?; // Depot

        for &customer in &route.customers {
            write!(file, " -> {}", customer)?;
        }

        writeln!(file, " -> 0")?; // Return to depot

        writeln!(file, "  Distance: {:.2}", route.distance)?;
        writeln!(
            file,
            "  Load: {:.2} / {:.2}",
            route.load, problem.vehicle_capacity
        )?;
        writeln!(file, "")?;
    }

    Ok(())
}

/// Calculate the load excess for a solution.
pub fn calculate_excess_load(solution: &Solution, problem: &Problem) -> f64 {
    let mut total_excess = 0.0;

    for route in &solution.routes {
        if route.load > problem.vehicle_capacity {
            total_excess += route.load - problem.vehicle_capacity;
        }
    }

    total_excess
}

/// Generate statistics about the search process.
pub struct SearchStatistics {
    pub iterations: u32,
    pub runtime: Duration,
    pub best_solution_cost: f64,
    pub best_solution_distance: f64,
    pub best_solution_is_feasible: bool,
    pub best_solution_routes: usize,
    pub average_population_size: usize,
    pub final_capacity_penalty: f64,
}

impl SearchStatistics {
    /// Format the statistics as a string.
    pub fn format(&self) -> String {
        format!(
            "Search Statistics:
- Iterations: {}
- Runtime: {}
- Best Solution Cost: {:.2}
- Best Solution Distance: {:.2}
- Best Solution Feasible: {}
- Best Solution Routes: {}
- Average Population Size: {}
- Final Capacity Penalty: {:.2}",
            self.iterations,
            format_duration(self.runtime),
            self.best_solution_cost,
            self.best_solution_distance,
            self.best_solution_is_feasible,
            self.best_solution_routes,
            self.average_population_size,
            self.final_capacity_penalty
        )
    }
}

/// Print solution visualization to console.
pub fn print_solution_visualization(solution: &Solution, problem: &Problem) {
    println!("Solution Visualization for {}", problem.name);
    println!("Total Distance: {:.2}", solution.distance);
    println!("Number of Routes: {}", solution.routes.len());
    println!();

    // Find max and min coordinates for scaling
    let mut min_x = f64::MAX;
    let mut min_y = f64::MAX;
    let mut max_x = f64::MIN;
    let mut max_y = f64::MIN;

    for node in &problem.nodes {
        min_x = min_x.min(node.x);
        min_y = min_y.min(node.y);
        max_x = max_x.max(node.x);
        max_y = max_y.max(node.y);
    }

    // Define visualization grid size
    let width = 80;
    let height = 25;

    // Create grid
    let mut grid = vec![vec![' '; width]; height];

    // Plot depot
    let depot = &problem.nodes[problem.depot_index];
    let depot_x = ((depot.x - min_x) / (max_x - min_x) * (width as f64 - 1.0)) as usize;
    let depot_y = ((depot.y - min_y) / (max_y - min_y) * (height as f64 - 1.0)) as usize;
    grid[depot_y][depot_x] = 'D';

    // Plot customers and routes with different symbols for each route
    let route_symbols = ['*', '+', 'x', '#', '@', '&', '%', '=', '^', '$'];

    for (r_idx, route) in solution.routes.iter().enumerate() {
        let symbol = route_symbols[r_idx % route_symbols.len()];

        for &customer in &route.customers {
            let node = &problem.nodes[customer];
            let x = ((node.x - min_x) / (max_x - min_x) * (width as f64 - 1.0)) as usize;
            let y = ((node.y - min_y) / (max_y - min_y) * (height as f64 - 1.0)) as usize;

            grid[y][x] = symbol;
        }
    }

    // Print the grid
    for row in &grid {
        for &cell in row {
            print!("{}", cell);
        }
        println!();
    }
    println!();

    // Print legend
    println!("Legend:");
    println!("D - Depot");
    for (r_idx, _) in solution.routes.iter().enumerate().take(route_symbols.len()) {
        println!("{} - Route #{}", route_symbols[r_idx], r_idx + 1);
    }
    println!();
}
