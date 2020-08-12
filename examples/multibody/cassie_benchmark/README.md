Cassie benchmark
----------------

This is a real-world example of a medium-sized robot with timing
tests for calculating its mass matrix, inverse dynamics, and
forward dynamics and their AutoDiff derivatives.

This gives us a straightforward way to measure local,
machine-specific, improvements in these basic multibody calculations
in Drake.

This program does not serve as a performance regression test as it has
no way to normalize for changes in hardware or other environmental
factors. Its primary purpose is to serve as a platform where
performance can be measured, code tweaked, and performance measured
again. Improvements observed on this benchmark on a particular machine
do not guarantee improvements on other robots or other machines.

## Full experiments

The following command will perform a benchmark experiment and
collect context information.

    $ bazel run //examples/multibody/cassie_benchmark:record_results

For best results, some conditions will need to be manually
controlled. These include:

* load average -- close as many other programs as practical
* cpu throttling -- varies with platform
  * Linux: https://github.com/google/benchmark#disabling-cpu-frequency-scaling

Bazel buries the results deep in its output tree. To copy them
somewhere more convenient, use this command outside of Bazel:

    $ examples/multibody/cassie_benchmark/copy_results_to [DIRECTORY]

Note that the results output is an ongoing effort to record the
conditions at the time the experiment was run, but it is not currently
complete. Still missing:

 * compiler version and build flags
 * git hash or similar source code version marker
 * platform information for unsupported platforms

## Configuring details of benchmark runs

To run individual benchmarks, alter output, etc., run the
benchmark program directly:

    $ bazel run //examples/multibody/cassie_benchmark:cassie_bench -- [ARGS]

Documentation for command line arguments is here:
https://github.com/google/benchmark#command-line

TODO(rpoyner-tri): automation and instructions for collecting and
storing baseline data, and for comparing experiment results to a
baseline.
