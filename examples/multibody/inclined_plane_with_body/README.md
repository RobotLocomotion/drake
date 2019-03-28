## Simulate slip/stick of a body (e.g., sphere or block) on an inclined plane.

### Description

This example simulates the motion of a rigid body B (e.g., a sphere or a block)
on an inclined-plane A.  You can simulate a uniform-density sphere B rolling 
down an inclined-plane A or simulate a block B that is sticking or slipping on 
an inclined-plane A.  The inclined-plane can be modeled as either an infinite 
half-space or as a finite box (so the block can fall off the inclined-plane).

### Building and running this example

First complete the one-time setup instructions, if you have not already done so:
https://drake.mit.edu/from_source.html#mandatory-platform-specific-instructions

Open a terminal in the `drake` workspace, and type commands as shown below.

Build the example like so:
```
bazel build //tools:drake_visualizer //examples/multibody/inclined_plane_with_body
```

Run the example like so:
```
bazel-bin/tools/drake_visualizer &
bazel-bin/examples/multibody/inclined_plane_with_body/inclined_plane_with_body
```

Alternatively,
to simulate body B as a block that has 4 contacting spheres welded to its
lowest four corners on a 30 degree inclined-plane A (modeled as a half-space), 
pass command line arguments to the executable by typing:
```
bazel-bin/examples/multibody/inclined_plane_with_body/inclined_plane_with_body -slope_degrees=30 -bodyB_type=block_with_4Spheres
```

To see a list of all possible command-line arguments:
```
bazel-bin/examples/multibody/inclined_plane_with_body/inclined_plane_with_body -help
```

### How are the coefficients of friction used?
1. A coefficient of friction is assigned separately to each object (a default
   value is assigned or you may assign a value via command-line arguments).
   Thereafter, a "combining equation" calculates the coefficient of friction
   by combining the coefficients of friction assigned to each object.
2. When time_step > 0 (fixed-time step), the coefficient of_kinetic_friction is 
   ignored since only the coefficient of static friction is used.
3. More information on the CoulombFriction class is in coulomb_friction.h.
