Runtime Performance Benchmarks for Geometry Operations
------------------------------------------------------

# Supported experiments

## IRIS in Configuration Space

  $ bazel run //geometry/optimization/benchmarking:iris_in_configuration_space_experiment -- --output_dir=foo

This benchmark is provided to help understand the implications of changes the
will impact the performance of the IrisInConfigurationSpace algorithm. It
should grow to include a number of our most important/relevant examples.

# Additional information

Documentation for command line arguments is here:
https://github.com/google/benchmark#command-line
