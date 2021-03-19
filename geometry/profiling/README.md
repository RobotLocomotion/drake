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
Compute contact surface between an anchored rigid bowl and a moving soft
ball. The rigid bowl is a realistic non-convex object, and the soft ball uses
a coarse tetrahedral mesh, which is typical in hydroelastic contact model.

To visualize the contact surface and pressure, run drake_visualizer and
configure Hydroelastic Contact Visualization plugin as follows:
1. From the top menu, click on: Plugins > Contacts > Configure Hydroelastic
 Contact
 Visualization.
2. Change "Maximum pressure" to a reasonably large number (for example, 4e7).
3. Check or uncheck "Render contact surface with pressure" as you prefer.
4. Check or uncheck "Render contact surface wireframe" as you prefer.
5. Click "OK".
6. Run this profiling example.

Instead of the default rigid bowl, it can optionally use a rigid ball, a
rigid box, a rigid cylinder or a rigid capsule. Instead of the default soft
ball, it can use a soft box, a soft cylinder or a soft capsule.

- default rigid bowl and default soft ball,
```
bazel-bin/geometry/profiling/contact_surface_rigid_bowl_soft_ball
```
- default rigid bowl with a soft box option,
```
bazel-bin/geometry/profiling/contact_surface_rigid_bowl_soft_ball --soft=box
```
- rigid box option and default soft ball,
```
bazel-bin/geometry/profiling/contact_surface_rigid_bowl_soft_ball --rigid=box
```
- general syntax with all options.
```
bazel-bin/geometry/profiling/contact_surface_rigid_bowl_soft_ball \
--rigid=[ball, bowl, box, capsule, cylinder] --soft=[ball, box, capsule, cylinder]
```
