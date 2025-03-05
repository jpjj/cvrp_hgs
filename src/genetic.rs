//! Genetic operators for the HGS-CVRP algorithm.

use crate::individual::Individual;
use crate::solution::Solution;
use rand::{seq::SliceRandom, thread_rng, Rng};
use std::collections::HashSet;

/// Implements the genetic operators (crossover, mutation) for the HGS-CVRP.
pub struct Genetic;

impl Genetic {
    /// Perform ordered crossover (OX) between two parent solutions.
    pub fn crossover(&self, parent1: &Individual, parent2: &Individual) -> Solution {
        let mut rng = thread_rng();

        let p1_tour = &parent1.solution.giant_tour;
        let p2_tour = &parent2.solution.giant_tour;

        if p1_tour.is_empty() || p2_tour.is_empty() {
            return Solution::new();
        }

        // Determine cutting points for OX
        let tour_size = p1_tour.len();
        let cut1 = rng.gen_range(0..tour_size);
        let cut2 = rng.gen_range(0..tour_size);

        let (start, end) = if cut1 <= cut2 {
            (cut1, cut2)
        } else {
            (cut2, cut1)
        };

        // Step 1: Create a new tour with placeholders
        let mut offspring_tour = vec![0; tour_size];

        // Step 2: Copy segment from first parent
        for i in start..=end {
            offspring_tour[i] = p1_tour[i];
        }

        // Create a set of customers already in the offspring
        let mut used = HashSet::new();
        for i in start..=end {
            used.insert(p1_tour[i]);
        }

        // Step 3: Fill remaining positions from the second parent
        let mut j = (end + 1) % tour_size;
        let mut p2_idx = (end + 1) % p2_tour.len();

        while used.len() < tour_size {
            if !used.contains(&p2_tour[p2_idx]) {
                offspring_tour[j] = p2_tour[p2_idx];
                used.insert(p2_tour[p2_idx]);
                j = (j + 1) % tour_size;
            }
            p2_idx = (p2_idx + 1) % p2_tour.len();
        }

        // Create a new solution from the offspring tour
        Self::create_solution_from_tour(offspring_tour, parent1.solution.routes.len())
    }

    /// Create a solution from a giant tour.
    fn create_solution_from_tour(giant_tour: Vec<usize>, route_count: usize) -> Solution {
        let mut solution = Solution::new();
        solution.giant_tour = giant_tour;
        // The Split algorithm will be applied separately to create routes
        solution
    }

    /// Implement a simple swap mutation operator.
    pub fn mutate(&self, individual: &mut Individual, mutation_rate: f64) {
        let mut rng = thread_rng();

        if individual.solution.giant_tour.is_empty() {
            return;
        }

        let tour_size = individual.solution.giant_tour.len();

        // For each position, apply mutation with probability = mutation_rate
        for i in 0..tour_size {
            if rng.gen::<f64>() < mutation_rate {
                // Select another random position
                let j = rng.gen_range(0..tour_size);

                // Swap customers
                individual.solution.giant_tour.swap(i, j);
            }
        }
    }
}
