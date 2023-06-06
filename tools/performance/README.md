# Tools support for performance analysis

This directory contains tools to help with careful performance measurement and
analysis of Drake programs.

Fully worked examples that integrate this tooling are available at:
- drake/geometry/benchmarking
- drake/multibody/benchmarking
- drake/solvers/benchmarking
- drake/systems/benchmarking

Some of the history of attempts to drive variance out of benchmark results is
captured in #13902.

TODO(rpoyner-tri): explain how to use compare.py from the googlebenchmark
package to compare stored results from different experiments.

## Tips

Fixtures can be used in a similar fashion to gtest. For more info see:
https://github.com/google/benchmark#fixtures.

One routine can be used to run a family of benchmarks by passing in a range of
arguments through the benchmarking state. In the `render_benchmark.cc` example,
arguments are used to vary the number of spheres rendered, the number of cameras
in the scene, and the image dimensions. For more info see:
https://github.com/google/benchmark#passing-arguments.

By default google-benchmark outputs times in nanoseconds; in many cases you'll
want milliseconds instead. For more info see:
https://github.com/google/benchmark#setting-the-time-unit.
