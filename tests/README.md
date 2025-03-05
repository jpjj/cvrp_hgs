# Tests for HGS-CVRP

This directory contains the test suite for the Hybrid Genetic Search for Capacitated Vehicle Routing Problem (HGS-CVRP) implementation. The tests are designed to verify the correctness of both individual components and the integration of the full algorithm.

## Test Organization

The tests are organized by module:

- `algorithm_integration_tests.rs`: Tests for the full algorithm integration
- `genetic_tests.rs`: Tests for genetic operators and population management
- `local_search_tests.rs`: Tests for individual local search neighborhoods
- `local_search_integration_tests.rs`: Tests for the local search system as a whole
- `solution_tests.rs`: Tests for the solution and route data structures
- `split_tests.rs`: Tests for the Split algorithm
- `utils_tests.rs`: Tests for utility functions

## Running the Tests

To run all tests:

```bash
cargo test
```

To run tests for a specific module:

```bash
cargo test --test local_search_tests
```

To run a specific test:

```bash
cargo test --test local_search_tests test_relocate_neighborhood
```

## Test Coverage

The test suite aims to cover:

1. **Correctness**: Verify that all components produce the expected results
2. **Edge Cases**: Test behavior with empty solutions, single customers, etc.
3. **Integration**: Ensure components work together correctly
4. **Performance**: Basic checks that the algorithm improves solutions over time

## Adding New Tests

When adding new functionality, please follow these guidelines for creating tests:

1. Place component tests in the appropriate module file
2. For new components, create a new test file following the established naming pattern
3. Include tests for normal operation, edge cases, and error conditions
4. Use the helper functions (e.g., `create_test_problem()`) to reduce code duplication
5. Keep tests focused and fast - avoid unnecessarily long-running tests

## Test Problem Generation

Several helper functions are available to create test problems:

- `create_test_problem()`: Creates a small test problem with a depot and 5 customers
- `create_complex_problem()`: Creates a larger problem with a grid of customers
- `create_moderate_problem()`: Creates a medium-sized problem for integration tests

These functions help maintain consistency across tests and reduce code duplication.