This directory contains examples for profiling with valgrind+callgrind
and kcachegrind.

# Profiling instructions.
Install `valgrind`, `kcachegrind`, and `graphviz`. For example on Ubuntu:
```
sudo apt-get install valgrind kcachegrind graphviz
```

Compile examples as release with debug info. For example:
```
bazel build --compilation_mode=opt --copt=-g //geometry/profiling:contact_surface_rigid_bowl_soft_ball
```
Run `valgrind` with `callgrind` to get profiling data in the file
`callgrind.out.*`. For example:
```
valgrind --tool=callgrind bazel-bin/geometry/profiling/contact_surface_rigid_bowl_soft_ball
```
Run `kcachegrind` to analyze profiling data. For example:
```
kcachegrind callgrind.out.19482
```

# Available Examples.

## contact_surface_rigid_bowl_soft_ball.cc:
Compute contact surface between an anchored rigid bowl and a moving compliant
ball. The rigid bowl is a realistic non-convex object, and the compliant
ball uses a coarse tetrahedral mesh, which is typical in hydroelastic 
contact model.

To visualize the contact surface and pressure, run drake_visualizer and
configure Hydroelastic Contact Visualization plugin as follows:
1. From the top menu, click on: `Plugins > Contacts > Configure Hydroelastic
 Contact Visualization.`
2. Change `Maximum pressure` to a reasonably large number (for example, 4e7).
3. Check or uncheck `Render contact surface with pressure` as you prefer.
4. Check or uncheck `Render contact surface edges` (wireframe) as you prefer.
5. Click `OK`.
6. Run this profiling example.

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
