This directory contains examples for profiling with valgrind+callgrind
and kcachegrind.

# Profiling instructions.

For installation instructions and an overview of profiling tools, refer to
https://drake.mit.edu/profiling.html.

# Available Examples.

## contact_surface_rigid_bowl_soft_ball.cc:
Compute contact surface between an anchored rigid bowl and a moving compliant
ball. The rigid bowl is a realistic non-convex object, and the compliant
ball uses a coarse tetrahedral mesh, which is typical in hydroelastic 
contact model.

To visualize the contact surface and pressure, you're out of luck because we
have removed support for the tool that actually shows it. Sorry.

Instead of using polygons to represent contact surfaces, it can use triangles.
- use triangles to represent contact surfaces,
```
bazel-bin/geometry/profiling/contact_surface_rigid_bowl_soft_ball --polygons=false
```

Instead of the default rigid bowl, it can optionally use a rigid ball, a
rigid box, a rigid cylinder or a rigid capsule. Instead of the default
compliant ball, it can use a compliant box, a compliant cylinder or a
compliant capsule.
- default rigid bowl and default compliant ball,
```
bazel-bin/geometry/profiling/contact_surface_rigid_bowl_soft_ball
```
- default rigid bowl with a compliant box option,
```
bazel-bin/geometry/profiling/contact_surface_rigid_bowl_soft_ball --compliant=box
```
- rigid box option and default compliant ball,
```
bazel-bin/geometry/profiling/contact_surface_rigid_bowl_soft_ball --rigid=box
```
- general syntax with all options.
```
bazel-bin/geometry/profiling/contact_surface_rigid_bowl_soft_ball \
--rigid=[ball, bowl, box, capsule, cylinder] --compliant=[ball, box, capsule, cylinder] \
--polygons=[true, false]
```
