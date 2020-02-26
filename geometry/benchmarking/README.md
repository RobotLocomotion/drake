This directory contains examples of using
[google-benchmark](https://github.com/google/benchmark) to benchmark
functions and [perf](https://perf.wiki.kernel.org/index.php/Main_Page) for more
detailed profiling.

# Benchmark infrastructure

## Arguments

One routine can be used to run a family of benchmarks by passing in a range of
arguments through the benchmarking state. In the `render_benchmark.cc` example,
arguments are used to vary the number of spheres rendered, the number of cameras
in the scene, and the image dimensions. For more info see:
https://github.com/google/benchmark#passing-arguments

## Time Units

By default google-benchmark outputs times in nanoseconds. In the
`render_benchmark.cc` example it is manually set to milliseconds. For more info
see: https://github.com/google/benchmark#setting-the-time-unit.

## Fixtures

Fixtures can be used in a similar fashion to gtest. In the
`render_benchmark.cc` example it is used for common render set up. For more
info see: https://github.com/google/benchmark#fixtures.

# Available Benchmarks

* [render_benchmark.cc](https://drake.mit.edu/doxygen_cxx/html/group__render__engine__benchmarks.html):
Benchmark program to help characterize the relative costs of different
RenderEngine implementations with varying scene complexity and rendering. It is
designed so users can assess the relative cost of the renderers on their own
hardware configuration, aiding in design decisions for understanding the cost of
renderer choice.
* [mesh_intersection_benchmark.cc](https://drake.mit.edu/doxygen_cxx/html/group__mesh__intersection__benchmarks.html):
Benchmark program to compare different mesh intersection optimizations with
varying mesh attributes and overlaps. It is targeted toward developers during
the process of optimizing the performance of hydroelastic contact and may be
removed once sufficient work has been done in that effort.

# Profile infrastructure

The profiling tool, [perf](https://perf.wiki.kernel.org/index.php/Main_Page),
can be run on the binary without any special compilation needed. Typically a
run is triggered using the command `perf record -g <binary>`, after which the
result can be viewed using the command `perf report -g`. Note that `sudo` may be required depending on the user's permission configuration.

## Arguments

For `perf`, the argument `-g` enables call-graph recording and display. Other
gflag arguments can be specified depending on the profile itself.

## Available Profiles

* [simple_contact_surface_profile.cc](https://drake.mit.edu/doxygen_cxx/html/group__simple__contact__surface__profile.html):
A simulation that illustrates the contact surfaces between various
configurations of soft and rigid meshes.

## Alternative Profilers

`perf` is an example of a sampling based profiler which is lightweight and fast,
but there are other profiling tools available. A good overview is available at
http://euccas.github.io/blog/20170827/cpu-profiling-tools-on-linux.html.
There are both advantages and disadvantages to using one tool over the other.
For example, this thread summarises a comparison between `perf` and an
instrumentation based profiler, `valgrind`:
https://news.ycombinator.com/item?id=12060362. The choice of profiler is
ultimately up to the user and what they want to investigate.
