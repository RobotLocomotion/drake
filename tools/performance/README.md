# Tools support for performance analysis

This directory contains tools to help with careful performance measurement and
analysis of Drake programs.

## Benchmarking tools

Included here are two tools to help with well-controlled benchmark experiments:

 * `record_results.py` -- run benchmark under bazel, record context and results
 * `benchmark_tool.py` -- outside-bazel tool for experiments and data handling

A fully worked example that integrates these two tools is available at
`drake/examples/multibody/cassie_benchmark`. Some of the history of attempts to
drive variance out of benchmark results is captured in #13902.

TODO(rpoyner-tri): explain how to use compare.py from the googlebenchmark
package to compare stored results from different experiments.
