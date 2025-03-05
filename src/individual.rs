//! Individual representation for the genetic algorithm population.

use crate::solution::Solution;
use std::cmp::Ordering;

/// An individual in the genetic algorithm population.
#[derive(Clone)]
pub struct Individual {
    /// The solution represented by this individual
    pub solution: Solution,
    /// The feasibility rank of the individual in the population
    pub rank_feasibility: usize,
    /// The diversity rank of the individual in the population
    pub rank_diversity: usize,
    /// The biased fitness based on both ranks
    pub biased_fitness: f64,
    /// Number of common pairs of consecutive customers with other individuals
    pub common_pairs: Vec<usize>,
}

impl Individual {
    /// Create a new individual from a solution.
    pub fn new(solution: Solution) -> Self {
        Individual {
            solution,
            rank_feasibility: 0,
            rank_diversity: 0,
            biased_fitness: 0.0,
            common_pairs: Vec::new(),
        }
    }

    /// Calculate the biased fitness of the individual based on ranks.
    pub fn calculate_biased_fitness(&mut self, elite_proportion: f64) {
        // Fitness is based on quality rank and diversity rank
        // Penalizing factor prevents the removal of elite solutions
        let penalizing_factor = 1.0 - elite_proportion;
        self.biased_fitness =
            self.rank_feasibility as f64 + penalizing_factor * self.rank_diversity as f64;
    }

    /// Calculate the average distance to the closest solutions.
    pub fn calculate_diversity_contribution(&self, closest_count: usize) -> f64 {
        if self.common_pairs.is_empty() || closest_count == 0 {
            return 0.0;
        }

        // Sort in ascending order (higher common pairs = closer solutions)
        let mut sorted_pairs = self.common_pairs.clone();
        sorted_pairs.sort_unstable_by(|a, b| b.cmp(a));

        // Take average of n closest
        let count = std::cmp::min(closest_count, sorted_pairs.len());
        let sum: usize = sorted_pairs.iter().take(count).sum();

        sum as f64 / count as f64
    }

    /// Check if this individual is a clone of another.
    pub fn is_clone_of(&self, other: &Individual) -> bool {
        if self.solution.giant_tour.len() != other.solution.giant_tour.len() {
            return false;
        }

        self.solution.giant_tour == other.solution.giant_tour
    }

    /// Calculate the number of common pairs of consecutive customers with another individual.
    pub fn calculate_common_pairs(&self, other: &Individual) -> usize {
        let mut common_count = 0;

        if self.solution.giant_tour.is_empty() || other.solution.giant_tour.is_empty() {
            return 0;
        }

        // Create a set of customer pairs in the other solution
        let mut other_pairs = std::collections::HashSet::new();

        for i in 0..other.solution.giant_tour.len() - 1 {
            let pair = (
                other.solution.giant_tour[i],
                other.solution.giant_tour[i + 1],
            );
            other_pairs.insert(pair);
        }

        // Count common pairs in this solution
        for i in 0..self.solution.giant_tour.len() - 1 {
            let pair = (self.solution.giant_tour[i], self.solution.giant_tour[i + 1]);
            if other_pairs.contains(&pair) {
                common_count += 1;
            }
        }

        common_count
    }

    /// Get the cost of the solution.
    pub fn get_cost(&self) -> f64 {
        self.solution.cost
    }

    /// Get the feasible cost of the solution.
    pub fn get_feasible_cost(&self) -> f64 {
        self.solution.get_feasible_cost()
    }

    /// Check if the solution is feasible.
    pub fn is_feasible(&self) -> bool {
        self.solution.is_feasible
    }
}

impl PartialEq for Individual {
    fn eq(&self, other: &Self) -> bool {
        self.solution.cost == other.solution.cost
    }
}

impl Eq for Individual {}

impl PartialOrd for Individual {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Individual {
    fn cmp(&self, other: &Self) -> Ordering {
        // For the genetic algorithm, we compare individuals based on their fitness
        self.biased_fitness
            .partial_cmp(&other.biased_fitness)
            .unwrap_or(Ordering::Equal)
    }
}
