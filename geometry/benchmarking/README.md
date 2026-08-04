Runtime Performance Benchmarks for Geometry Operations
------------------------------------------------------

# Supported experiments

## render

```
$ bazel run //geometry/benchmarking:render_experiment -- --output_dir=foo
```

Benchmark program to help characterize the relative costs of different
RenderEngine implementations with varying scene complexity and rendering. It is
designed so users can assess the relative cost of the renderers on their own
hardware configuration, aiding in design decisions for understanding the cost of
renderer choice.

## mesh_intersection

```
$ bazel run //geometry/benchmarking:mesh_intersection_experiment -- --output_dir=foo
```

Benchmark program to evaluate bounding volume hierarchy impact on mesh-mesh
intersections across varying mesh attributes and overlaps. It is targeted toward
developers during the process of optimizing the performance of hydroelastic
contact and may be removed once sufficient work has been done in that effort.

## iris_np

Note: This benchmark requires [SNOPT](https://drake.mit.edu/bazel.html#snopt).

```
$ bazel run //geometry/benchmarking:iris_np_experiment -- --output_dir=foo
```

This benchmark is provided to help understand the implications of changes the
will impact the performance of the IrisNP algorithm. It should grow to include a
number of our most important/relevant examples.

# Additional information

Documentation for command line arguments is here:
https://github.com/google/benchmark#command-line
