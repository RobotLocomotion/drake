Cassie benchmark
----------------

This is a real-world example of a medium-sized robot with timing
tests for calculating its mass matrix, inverse dynamics, and
forward dynamics and their AutoDiff derivatives.

This gives us a straightforward way to measure improvements in
these basic multibody calculations in Drake.

The following command will perform a benchmark experiment and
collect context information.

    $ bazel run //examples/multibody/cassie_benchmark:record_results

To run individual benchmarks, alter output, etc., run the
benchmark program directly:

    $ bazel run //examples/multibody/cassie_benchmark:record_results -- [ARGS]

Documentation for command line arguments is here:
https://github.com/google/benchmark#command-line

For best results, some conditions will need to be manually
controlled. These include:

* load average -- close as many other programs as practical
* cpu throttling -- varies with platform
  * Linux: https://github.com/google/benchmark#disabling-cpu-frequency-scaling

TODO(rpoyner-tri): automation and instructions for collecting and
storing baseline data, and for comparing experiment results to a
baseline.
