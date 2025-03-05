//! Population management for the genetic algorithm.

use crate::individual::Individual;
use crate::problem::Problem;
use crate::solution::Solution;
use crate::{config::Config, individual};
use rand::{seq::SliceRandom, Rng};
use std::collections::HashSet;

/// Manages the population of individuals for the genetic algorithm.
pub struct Population {
    /// Feasible individuals
    pub feasible_individuals: Vec<Individual>,
    /// Infeasible individuals
    pub infeasible_individuals: Vec<Individual>,
    /// The penalty coefficient for capacity violations
    pub capacity_penalty: f64,
    /// Minimum population size
    pub min_pop_size: usize,
    /// Maximum population size before survivor selection
    pub max_pop_size: usize,
    /// Number of closest solutions to consider for diversity
    pub n_closest: usize,
    /// Target ratio of feasible individuals
    pub target_feasible_ratio: f64,
    /// Number of elite individuals to preserve
    pub n_elite: usize,
}

impl Population {
    /// Create a new population with the given configuration.
    pub fn new(config: &Config) -> Self {
        Population {
            feasible_individuals: Vec::with_capacity(config.min_pop_size + config.generation_size),
            infeasible_individuals: Vec::with_capacity(
                config.min_pop_size + config.generation_size,
            ),
            capacity_penalty: config.initial_capacity_penalty,
            min_pop_size: config.min_pop_size,
            max_pop_size: config.min_pop_size + config.generation_size,
            n_closest: config.n_closest,
            target_feasible_ratio: config.target_feasible_ratio,
            n_elite: config.n_elite,
        }
    }

    /// Initialize the population with random individuals.
    pub fn initialize(&mut self, problem: &Problem, config: &Config) {
        let initial_size = 4 * self.min_pop_size;

        for _ in 0..initial_size {
            // Generate a random giant tour
            let mut giant_tour: Vec<usize> = (0..problem.get_customer_count()).collect();
            let mut rng = rand::thread_rng();
            giant_tour.shuffle(&mut rng);

            // Create a solution from the giant tour
            let mut solution = Solution::from_giant_tour(giant_tour, problem);

            // Evaluate the solution
            solution.evaluate(problem, self.capacity_penalty);

            // Create an individual and add to the appropriate subpopulation
            let individual = Individual::new(solution);
            self.insert_individual(individual);
        }

        // Update ranks and diversity measures
        self.update_ranks();
    }

    /// Insert a new individual into the appropriate subpopulation.
    pub fn insert_individual(&mut self, individual: Individual) {
        if individual.is_feasible() {
            self.feasible_individuals.push(individual);
        } else {
            self.infeasible_individuals.push(individual);
        }
    }

    /// Update the ranks of all individuals in the population.
    pub fn update_ranks(&mut self) {
        self.update_feasibility_ranks();
        self.update_diversity_measures();
        self.update_biased_fitness();
    }

    /// Update the feasibility ranks of all individuals.
    fn update_feasibility_ranks(&mut self) {
        // Sort feasible individuals by their cost
        self.feasible_individuals
            .sort_by(|a, b| a.get_cost().partial_cmp(&b.get_cost()).unwrap());

        // Assign ranks
        for (i, individual) in self.feasible_individuals.iter_mut().enumerate() {
            individual.rank_feasibility = i;
        }

        // Sort infeasible individuals by their cost
        self.infeasible_individuals
            .sort_by(|a, b| a.get_cost().partial_cmp(&b.get_cost()).unwrap());

        // Assign ranks
        for (i, individual) in self.infeasible_individuals.iter_mut().enumerate() {
            individual.rank_feasibility = i;
        }
    }

    /// Update the diversity measures for all individuals.
    fn update_diversity_measures(&mut self) {
        // First, clear previous common pairs data
        for individual in self.feasible_individuals.iter_mut() {
            individual.common_pairs.clear();
        }

        for individual in self.infeasible_individuals.iter_mut() {
            individual.common_pairs.clear();
        }

        // Calculate common pairs for feasible individuals
        self.calculate_common_pairs(true);

        // Calculate common pairs for infeasible individuals
        self.calculate_common_pairs(false);

        // Assign diversity ranks for feasible individuals
        self.assign_diversity_ranks(true);

        // Assign diversity ranks for infeasible individuals
        self.assign_diversity_ranks(false);
    }

    /// Calculate common pairs between all individuals in a subpopulation.
    fn calculate_common_pairs(&mut self, feasible: bool) {
        let individuals = match feasible {
            true => &mut self.feasible_individuals,
            false => &mut self.infeasible_individuals,
        };
        let count = individuals.len();

        for i in 0..count {
            individuals[i].common_pairs = vec![0; count];

            for j in 0..count {
                if i != j {
                    let common = individuals[i].calculate_common_pairs(&individuals[j]);
                    individuals[i].common_pairs[j] = common;
                }
            }
        }
    }

    /// Assign diversity ranks based on the distance to the closest individuals.
    fn assign_diversity_ranks(&mut self, feasible: bool) {
        let individuals = match feasible {
            true => &mut self.feasible_individuals,
            false => &mut self.infeasible_individuals,
        };
        if individuals.is_empty() {
            return;
        }

        // Calculate diversity contributions
        let mut diversity_values: Vec<(usize, f64)> = individuals
            .iter()
            .enumerate()
            .map(|(i, individual)| {
                (
                    i,
                    individual.calculate_diversity_contribution(self.n_closest),
                )
            })
            .collect();

        // Sort by diversity (higher diversity = lower rank)
        diversity_values.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

        // Assign ranks
        for (rank, (index, _)) in diversity_values.iter().enumerate() {
            individuals[*index].rank_diversity = rank;
        }
    }

    /// Update the biased fitness of all individuals.
    fn update_biased_fitness(&mut self) {
        let elite_prop = self.n_elite as f64 / self.get_pop_size() as f64;

        for individual in self.feasible_individuals.iter_mut() {
            individual.calculate_biased_fitness(elite_prop);
        }

        for individual in self.infeasible_individuals.iter_mut() {
            individual.calculate_biased_fitness(elite_prop);
        }
    }

    /// Get the total population size.
    pub fn get_pop_size(&self) -> usize {
        self.feasible_individuals.len() + self.infeasible_individuals.len()
    }

    /// Select parents from the population using binary tournament selection.
    pub fn select_parents(&self) -> (&Individual, &Individual) {
        let mut rng = rand::thread_rng();

        // First parent
        let parent1 = self.binary_tournament_selection(&mut rng);

        // Second parent (ensure different from first)
        let mut parent2 = self.binary_tournament_selection(&mut rng);
        while std::ptr::eq(parent1, parent2) {
            parent2 = self.binary_tournament_selection(&mut rng);
        }

        (parent1, parent2)
    }

    /// Perform binary tournament selection.
    fn binary_tournament_selection<R: Rng>(&self, rng: &mut R) -> &Individual {
        // Select a subpopulation
        let use_feasible = if self.feasible_individuals.is_empty() {
            false
        } else if self.infeasible_individuals.is_empty() {
            true
        } else {
            rng.gen_bool(0.5)
        };

        let subpop = if use_feasible {
            &self.feasible_individuals
        } else {
            &self.infeasible_individuals
        };

        if subpop.is_empty() {
            panic!("Cannot select from empty population");
        }

        // Select two individuals randomly
        let idx1 = rng.gen_range(0..subpop.len());
        let mut idx2 = rng.gen_range(0..subpop.len());

        // Ensure they are different
        while idx1 == idx2 && subpop.len() > 1 {
            idx2 = rng.gen_range(0..subpop.len());
        }

        // Return the one with better fitness
        if subpop[idx1].biased_fitness <= subpop[idx2].biased_fitness {
            &subpop[idx1]
        } else {
            &subpop[idx2]
        }
    }

    /// Check if we should perform survivor selection.
    pub fn should_manage_size(&self) -> bool {
        self.feasible_individuals.len() > self.max_pop_size
            || self.infeasible_individuals.len() > self.max_pop_size
    }

    /// Select survivors to maintain population size.
    pub fn select_survivors(&mut self) {
        self.select_survivors_for_subpop(true);
        self.select_survivors_for_subpop(false);
    }

    /// Select survivors for a subpopulation.
    fn select_survivors_for_subpop(&mut self, feasible: bool) {
        let individuals = match feasible {
            true => &mut self.feasible_individuals,
            false => &mut self.infeasible_individuals,
        };
        if individuals.len() <= self.min_pop_size {
            return;
        }

        // Sort by biased fitness
        individuals.sort_by(|a, b| a.biased_fitness.partial_cmp(&b.biased_fitness).unwrap());

        // Find and remove clones first
        let mut to_remove = HashSet::new();

        for i in 0..individuals.len() {
            if to_remove.contains(&i) {
                continue;
            }

            for j in (i + 1)..individuals.len() {
                if !to_remove.contains(&j) && individuals[i].is_clone_of(&individuals[j]) {
                    to_remove.insert(j);

                    if individuals.len() - to_remove.len() <= self.min_pop_size {
                        break;
                    }
                }
            }

            if individuals.len() - to_remove.len() <= self.min_pop_size {
                break;
            }
        }

        // If we still need to remove more, remove worst individuals by biased fitness
        let mut i = individuals.len() - 1;
        while individuals.len() - to_remove.len() > self.min_pop_size && i > 0 {
            if !to_remove.contains(&i) {
                to_remove.insert(i);
            }
            i -= 1;
        }

        // Remove individuals in reverse order to maintain indices
        let mut indices: Vec<_> = to_remove.into_iter().collect();
        indices.sort_unstable_by(|a, b| b.cmp(a));

        for idx in indices {
            individuals.remove(idx);
        }
    }

    /// Adjust the capacity penalty parameter.
    pub fn adjust_penalties(&mut self) {
        // Calculate current feasible ratio
        let total = self.get_pop_size();
        let feasible_count = self.feasible_individuals.len();

        if total == 0 {
            return;
        }

        let current_ratio = feasible_count as f64 / total as f64;

        // Adjust penalty
        if current_ratio < self.target_feasible_ratio {
            // Increase penalty to favor feasible solutions
            self.capacity_penalty *= 1.2;
        } else {
            // Decrease penalty to explore more infeasible solutions
            self.capacity_penalty /= 1.2;
        }

        // Ensure penalty doesn't get too small
        self.capacity_penalty = self.capacity_penalty.max(0.1);
    }

    /// Get the best feasible solution in the population.
    pub fn get_best_feasible_solution(&self) -> Option<&Solution> {
        self.feasible_individuals
            .iter()
            .min_by(|a, b| {
                a.get_cost()
                    .partial_cmp(&b.get_cost())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|x| &x.solution)
    }
}
