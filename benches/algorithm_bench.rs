//! Benchmarks for the HGS-CVRP algorithm.

#[cfg(feature = "bench")]
extern crate criterion;

#[cfg(feature = "bench")]
use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use cvrp_hgs::config::Config;
use cvrp_hgs::problem::{Node, Problem};
use cvrp_hgs::HgsAlgorithm;
use std::time::Duration;

/// Create a benchmark problem of specified size.
fn create_benchmark_problem(size: usize) -> Problem {
    let mut nodes = Vec::new();

    // Depot
    nodes.push(Node::new(0, 0.0, 0.0, 0.0, true));

    // Customers in a grid arrangement
    let grid_size = (size as f64).sqrt().ceil() as usize;
    for i in 1..=size {
        let row = (i - 1) / grid_size;
        let col = (i - 1) % grid_size;
        let x = col as f64 * 10.0;
        let y = row as f64 * 10.0;
        nodes.push(Node::new(i, x, y, 1.0, false));
    }

    Problem::new(
        format!("BenchProblem_{}", size),
        nodes,
        0,
        10.0,
        Some((size / 5).max(1)),
    )
}

#[cfg(feature = "bench")]
fn benchmark_initialization(c: &mut Criterion) {
    let mut group = c.benchmark_group("initialization");

    for size in [50, 100, 200].iter() {
        group.bench_with_input(BenchmarkId::from_parameter(size), size, |b, &size| {
            let problem = create_benchmark_problem(size);
            let config = Config::new().with_min_pop_size(25).with_generation_size(40);

            b.iter(|| {
                let mut algorithm = HgsAlgorithm::new(problem.clone(), config.clone());
                algorithm.initialize();
            });
        });
    }

    group.finish();
}

#[cfg(feature = "bench")]
fn benchmark_local_search(c: &mut Criterion) {
    let mut group = c.benchmark_group("local_search");

    for size in [50, 100, 200].iter() {
        group.bench_with_input(BenchmarkId::from_parameter(size), size, |b, &size| {
            let problem = create_benchmark_problem(size);
            let config = Config::new().with_min_pop_size(25).with_generation_size(40);

            let mut algorithm = HgsAlgorithm::new(problem.clone(), config.clone());
            algorithm.initialize();

            // Get a solution to improve
            let individual = &algorithm.population.feasible_individuals[0];
            let mut solution = individual.solution.clone();

            b.iter(|| {
                let mut solution_clone = solution.clone();
                algorithm
                    .local_search
                    .educate(&mut solution_clone, &problem, 1.0);
            });
        });
    }

    group.finish();
}

#[cfg(feature = "bench")]
fn benchmark_convergence(c: &mut Criterion) {
    let mut group = c.benchmark_group("convergence");
    group.measurement_time(Duration::from_secs(30));

    for size in [50, 100].iter() {
        group.bench_with_input(BenchmarkId::from_parameter(size), size, |b, &size| {
            let problem = create_benchmark_problem(size);
            let config = Config::new()
                .with_min_pop_size(25)
                .with_generation_size(40)
                .with_max_iterations_without_improvement(100)
                .with_time_limit(Duration::from_secs(10));

            b.iter(|| {
                let mut algorithm = HgsAlgorithm::new(problem.clone(), config.clone());
                algorithm.run();
            });
        });
    }

    group.finish();
}

#[cfg(feature = "bench")]
criterion_group!(
    benches,
    benchmark_initialization,
    benchmark_local_search,
    benchmark_convergence
);

#[cfg(feature = "bench")]
criterion_main!(benches);
