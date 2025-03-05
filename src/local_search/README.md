# Local Search Module

This directory contains the local search components of the Hybrid Genetic Search algorithm for CVRP.

## Organization

The code has been organized into separate files for better maintainability:

- `mod.rs`: Contains the main `LocalSearch` struct and common functionality
- `utils.rs`: Shared utility functions for all local search operations
- `relocate.rs`: Implementation of the Relocate neighborhood
- `swap.rs`: Implementation of the Swap neighborhood
- `two_opt.rs`: Implementation of the 2-Opt neighborhood (intra-route)
- `two_opt_star.rs`: Implementation of the 2-Opt* neighborhood (inter-route)
- `swap_star.rs`: Implementation of the SWAP* neighborhood (Vidal's contribution)

## Local Search Neighborhoods

### Relocate
Moves a customer from one route to another at the best insertion position.

### Swap
Exchanges two customers between different routes.

### 2-Opt
Reverses a segment within a route to remove crossing edges.

### 2-Opt*
Exchanges the tails of two routes after specified cutting points.

### SWAP*
An advanced neighborhood that swaps customers between routes but allows them to be
inserted at their best positions, not necessarily at the positions where the original
customers were removed.

## Implementation Details

All neighborhood operations follow a common pattern:
1. Check if the move has been tried before using the timestamp mechanism
2. Evaluate the move to calculate the potential cost change
3. Apply the move if it leads to improvement
4. Update timestamps and re-evaluate the solution

The SWAP* neighborhood uses route sector pruning to reduce the search space,
only considering routes with intersecting polar sectors.