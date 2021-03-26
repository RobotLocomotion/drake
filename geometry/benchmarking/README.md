This directory contains examples of using
[google-benchmark](https://github.com/google/benchmark) to benchmark
functions.

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

* [render_benchmark.cc](./render_benchmark.cc):
Benchmark program to help characterize the relative costs of different
RenderEngine implementations with varying scene complexity and rendering. It is
designed so users can assess the relative cost of the renderers on their own
hardware configuration, aiding in design decisions for understanding the cost of
renderer choice.
* [mesh_intersection_benchmark.cc](./mesh_intersection_benchmark.cc):
Benchmark program to evaluate bounding volume hierarchy impact on mesh-mesh
intersections across varying mesh attributes and overlaps. It is targeted toward
developers during the process of optimizing the performance of hydroelastic
contact and may be removed once sufficient work has been done in that effort.
