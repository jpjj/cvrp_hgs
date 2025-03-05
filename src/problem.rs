//! Problem definition and data structures for CVRP.

use serde::{Deserialize, Serialize};
use std::f64;
use std::fs::File;
use std::io::{self, BufRead};
use std::path::Path;

/// Represents a node (customer or depot) in the CVRP.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Node {
    pub id: usize,
    pub x: f64,
    pub y: f64,
    pub demand: f64,
    pub is_depot: bool,
}

impl Node {
    /// Create a new node.
    pub fn new(id: usize, x: f64, y: f64, demand: f64, is_depot: bool) -> Self {
        Node {
            id,
            x,
            y,
            demand,
            is_depot,
        }
    }

    /// Calculate the Euclidean distance between two nodes.
    pub fn distance(&self, other: &Node) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

/// Represents a CVRP problem instance.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Problem {
    pub name: String,
    pub nodes: Vec<Node>,
    pub depot_index: usize,
    pub vehicle_capacity: f64,
    pub max_vehicles: Option<usize>,
    pub distance_matrix: Vec<Vec<f64>>,
}

impl Problem {
    /// Create a new CVRP problem.
    pub fn new(
        name: String,
        nodes: Vec<Node>,
        depot_index: usize,
        vehicle_capacity: f64,
        max_vehicles: Option<usize>,
    ) -> Self {
        let distance_matrix = Self::compute_distance_matrix(&nodes);

        Problem {
            name,
            nodes,
            depot_index,
            vehicle_capacity,
            max_vehicles,
            distance_matrix,
        }
    }

    /// Calculate the distance between two customer indices.
    pub fn get_distance(&self, from: usize, to: usize) -> f64 {
        self.distance_matrix[from][to]
    }

    /// Get the number of customers (excluding the depot).
    pub fn get_customer_count(&self) -> usize {
        self.nodes.len() - 1
    }

    /// Get the depot node.
    pub fn get_depot(&self) -> &Node {
        &self.nodes[self.depot_index]
    }

    /// Generate the full distance matrix for all nodes.
    fn compute_distance_matrix(nodes: &[Node]) -> Vec<Vec<f64>> {
        let n = nodes.len();
        let mut matrix = vec![vec![0.0; n]; n];

        for i in 0..n {
            for j in 0..n {
                if i != j {
                    matrix[i][j] = nodes[i].distance(&nodes[j]);
                }
            }
        }

        matrix
    }

    /// Calculate the center of all customer locations.
    pub fn calculate_center(&self) -> (f64, f64) {
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut count = 0;

        for node in &self.nodes {
            if !node.is_depot {
                sum_x += node.x;
                sum_y += node.y;
                count += 1;
            }
        }

        if count > 0 {
            (sum_x / count as f64, sum_y / count as f64)
        } else {
            (0.0, 0.0)
        }
    }

    /// Load a problem from a file.
    pub fn from_file<P: AsRef<Path>>(path: P) -> io::Result<Self> {
        let file = File::open(path)?;
        let reader = io::BufReader::new(file);
        let mut lines = reader.lines();

        // Parse problem name
        let name = lines.next().unwrap()?.trim().to_string();

        // Parse vehicle information
        let vehicle_info = lines.next().unwrap()?;
        let parts: Vec<&str> = vehicle_info.split_whitespace().collect();
        let vehicle_capacity = parts[0].parse::<f64>().unwrap();
        let max_vehicles = if parts.len() > 1 {
            Some(parts[1].parse::<usize>().unwrap())
        } else {
            None
        };

        // Parse node information
        let mut nodes = Vec::new();
        let mut depot_index = 0;

        for (i, line_result) in lines.enumerate() {
            let line = line_result?;
            let parts: Vec<&str> = line.split_whitespace().collect();

            if parts.len() >= 4 {
                let id = i;
                let x = parts[1].parse::<f64>().unwrap();
                let y = parts[2].parse::<f64>().unwrap();
                let demand = parts[3].parse::<f64>().unwrap();
                let is_depot = demand == 0.0;

                if is_depot {
                    depot_index = id;
                }

                nodes.push(Node::new(id, x, y, demand, is_depot));
            }
        }

        Ok(Problem::new(
            name,
            nodes,
            depot_index,
            vehicle_capacity,
            max_vehicles,
        ))
    }
}
