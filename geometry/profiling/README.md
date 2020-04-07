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

* contact_surface_rigid_bowl_soft_ball.cc:
Compute contact surface between an anchored rigid bowl and a dynamic soft
ball. The rigid bowl is a realistic non-convex object, and the soft ball uses
a coarse tetrahedral mesh, which is typical in hydroelastic contact model.
