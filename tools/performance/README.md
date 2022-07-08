# Tools support for performance analysis

This directory contains tools to help with careful performance measurement and
analysis of Drake programs.

Fully worked examples that integrate this tooling are available at:
- drake/geometry/benchmarking   (still TODO; see below)
- drake/multibody/benchmarking
- drake/solvers/benchmarking
- drake/systems/benchmarking

Some of the history of attempts to drive variance out of benchmark results is
captured in #13902.

TODO(jwnimmer-tri): Port //geometry/benchmarking to use this tooling as well.

TODO(rpoyner-tri): explain how to use compare.py from the googlebenchmark
package to compare stored results from different experiments.
