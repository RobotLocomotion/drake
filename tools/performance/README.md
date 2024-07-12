# Tools support for performance analysis

This directory contains libraries, tools, and build scripts to help with
careful performance measurement and analysis of Drake programs.

## Benchmarking

The files listed below help with benchmarking of Drake programs.

- benchmark_tool.py
- defs.bzl
- fixture*.{cc,h}
- gflags_main.cc

Fully worked examples that integrate this tooling are available at:
- drake/geometry/benchmarking
- drake/multibody/benchmarking
- drake/solvers/benchmarking
- drake/systems/benchmarking

Some of the history of attempts to drive variance out of benchmark results is
captured in #13902.

TODO(rpoyner-tri): explain how to use compare.py from the googlebenchmark
package to compare stored results from different experiments.

### Tips

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


## Profiling

The files listed below help with statistical profiling of subsections of larger
programs. This kind of selection can be helpful when demonstrating a bottleneck
requires elaborate setup. See these files:

- perf_controlled_record.py
- perf_controller*.{cc,h}
- perf_controller*.py

These files configure Linux `perf` such that the target program can include
instructions to turn sampling on and off. In this way, subprograms of interest
can be measured without extensive rewriting.

The perf_controlled_record.py script configures `perf` and launches a
user-supplied profiling target command. The target program should use either
the C++ or Python PerfController modules (or both) to turn sampling on and off
within the program.

The supplied example target programs (perf_controller*test_target*) allow a
manual demonstration of the technique. Suppose we want to profile some
subprograms, and view data in `hotspot` (apt install hotspot). Hotspot requires
DWARF call graph data, so `perf` needs to be told that by an extra option. For
the C++ target, it looks like this:

    $ bazel build //tools/performance/...
    $ ./tools/performance/perf_controlled_record.py \
      --extra_perf_args="--call-graph=dwarf"  --  \
      ./bazel-bin/tools/performance/perf_controller_test_target
    $ hotspot  # loads perf.data by default

In the data, it should be clear that only subprograms `busy_work_2` and
`busy_work_4` are measured.

Measuring the python target is essentially the same:

    $ ./tools/performance/perf_controlled_record.py \
      --extra_perf_args="--call-graph=dwarf"  -- \
      ./bazel-bin/tools/performance/perf_controller_py_test_target
    $ hotspot  # loads perf.data by default

When looking at data from a python program, recall that `perf` is meant for
measuring native code, so it has no way to extract Python function names. It
should be possible see that the native-code internals of `busy_work_2`
(math_cos) and `busy_work_4` are (math_asin) are measured.
