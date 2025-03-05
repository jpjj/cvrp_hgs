//! Solution representation for the CVRP.

use crate::problem::Problem;
use serde::{Deserialize, Serialize};
use std::fmt;

/// Represents a route in a CVRP solution.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Route {
    /// The sequence of customer indices (excluding the depot)
    pub customers: Vec<usize>,
    /// The total load of the route
    pub load: f64,
    /// The total distance of the route
    pub distance: f64,
    /// Has this route been modified since last evaluation
    pub modified: bool,
}

impl Route {
    /// Create a new, empty route.
    pub fn new() -> Self {
        Route {
            customers: Vec::new(),
            load: 0.0,
            distance: 0.0,
            modified: true,
        }
    }

    /// Create a route with a single customer.
    pub fn with_customer(customer: usize, load: f64, distance_from_depot: f64) -> Self {
        let mut route = Route::new();
        route.customers.push(customer);
        route.load = load;
        route.distance = distance_from_depot * 2.0; // From depot to customer and back
        route.modified = true;
        route
    }

    /// Calculate the total distance of this route.
    pub fn calculate_distance(&mut self, problem: &Problem) {
        if !self.modified {
            return;
        }

        let depot_index = problem.depot_index;
        let mut total_distance = 0.0;

        if self.customers.is_empty() {
            self.distance = 0.0;
            self.modified = false;
            return;
        }

        // Distance from depot to first customer
        total_distance += problem.get_distance(depot_index, self.customers[0]);

        // Distance between consecutive customers
        for i in 0..self.customers.len() - 1 {
            total_distance += problem.get_distance(self.customers[i], self.customers[i + 1]);
        }

        // Distance from last customer back to depot
        total_distance +=
            problem.get_distance(self.customers[self.customers.len() - 1], depot_index);

        self.distance = total_distance;
        self.modified = false;
    }

    /// Calculate the total load of this route.
    pub fn calculate_load(&mut self, problem: &Problem) {
        if !self.modified {
            return;
        }

        let mut total_load = 0.0;

        for &customer in &self.customers {
            total_load += problem.nodes[customer].demand;
        }

        self.load = total_load;
    }

    /// Check if the route is empty.
    pub fn is_empty(&self) -> bool {
        self.customers.is_empty()
    }

    /// Check if the route exceeds the vehicle capacity.
    pub fn exceeds_capacity(&self, capacity: f64) -> bool {
        self.load > capacity
    }

    /// Get the load excess beyond the vehicle capacity.
    pub fn get_excess_load(&self, capacity: f64) -> f64 {
        if self.load > capacity {
            self.load - capacity
        } else {
            0.0
        }
    }
}

/// Represents a complete solution to a CVRP instance.
#[derive(Clone, Serialize, Deserialize)]
pub struct Solution {
    /// The list of routes
    pub routes: Vec<Route>,
    /// The total cost of the solution (distance + capacity violation penalties)
    pub cost: f64,
    /// The total raw distance of the solution
    pub distance: f64,
    /// The total capacity violation across all routes
    pub excess_capacity: f64,
    /// Is this solution feasible (no capacity violations)
    pub is_feasible: bool,
    /// The giant tour representation (sequence of all customers without route delimiters)
    pub giant_tour: Vec<usize>,
}

impl Solution {
    /// Create a new, empty solution.
    pub fn new() -> Self {
        Solution {
            routes: Vec::new(),
            cost: 0.0,
            distance: 0.0,
            excess_capacity: 0.0,
            is_feasible: true,
            giant_tour: Vec::new(),
        }
    }

    /// Create a solution with a given giant tour.
    pub fn from_giant_tour(giant_tour: Vec<usize>, problem: &Problem) -> Self {
        let mut solution = Solution::new();
        solution.giant_tour = giant_tour;

        // Split will be used to convert the giant tour into routes
        // This is handled separately in the split module

        solution
    }

    /// Evaluate the solution, calculating its cost and feasibility.
    pub fn evaluate(&mut self, problem: &Problem, capacity_penalty: f64) {
        let mut total_distance = 0.0;
        let mut total_excess = 0.0;

        for route in &mut self.routes {
            route.calculate_distance(problem);
            route.calculate_load(problem);

            total_distance += route.distance;
            total_excess += route.get_excess_load(problem.vehicle_capacity);
        }

        self.distance = total_distance;
        self.excess_capacity = total_excess;
        self.is_feasible = total_excess <= 1e-10;
        self.cost = total_distance + capacity_penalty * total_excess;
    }

    /// Update the giant tour from the routes.
    pub fn update_giant_tour(&mut self) {
        self.giant_tour.clear();

        for route in &self.routes {
            self.giant_tour.extend(&route.customers);
        }
    }

    /// Get the cost of the solution considering only feasible components.
    pub fn get_feasible_cost(&self) -> f64 {
        if self.is_feasible {
            self.distance
        } else {
            f64::INFINITY
        }
    }

    /// Get the number of routes.
    pub fn get_route_count(&self) -> usize {
        self.routes.len()
    }
}

impl fmt::Debug for Solution {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Solution:")?;
        writeln!(f, "  Cost: {:.2}", self.cost)?;
        writeln!(f, "  Distance: {:.2}", self.distance)?;
        writeln!(f, "  Excess Capacity: {:.2}", self.excess_capacity)?;
        writeln!(f, "  Feasible: {}", self.is_feasible)?;
        writeln!(f, "  Routes: {}", self.routes.len())?;

        for (i, route) in self.routes.iter().enumerate() {
            writeln!(
                f,
                "  Route {}: {:?} (Load: {:.2}, Distance: {:.2})",
                i, route.customers, route.load, route.distance
            )?;
        }

        Ok(())
    }
}
