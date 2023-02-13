Runtime Performance Benchmarks for Dynamical Systems
----------------------------------------------------

## Supported experiments

On Ubuntu, the following commands will build code and save result data
to a user supplied directory, under relatively controlled conditions:

    $ bazel run //systems/benchmarking:framework_experiment -- --output_dir=trial1

    $ bazel run //systems/benchmarking:multilayer_perceptron_experiment -- --output_dir=trial2

## Additional information

Documentation for command line arguments is here:
https://github.com/google/benchmark#command-line
