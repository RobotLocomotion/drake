## Simulate slip/stick of a body (e.g., sphere or block) on an inclined plane.

### Description
This example simulates the motion of a rigid body B (e.g., a sphere or a block)
on an inclined-plane A (which may be an infinite half-space or a finite box).
You can simulate a uniform-density sphere B rolling down an inclined-plane A or
simulate a block B that is sticking or slipping on an inclined-plane A.  The
inclined-plane can be modeled as either an infinite half-space or as a
finite-sized box (so the block can fall off the inclined-plane).

### How do I run the visualizer?
If the visualizer has already been built, you can run it by opening a terminal,
change to the drake directory, and at the operating system command prompt, type:
```
./bazel-bin/tools/drake_visualizer &
```

### How do I build the visualizer?
If the visualizer needs to be built, open a terminal, change to the drake
directory, and type something like the following at the operating system prompt:
```
bazel run //tools:drake_visualizer
```
Note: This requires that bazel (build tool) has been installed.  If this command
is successful, you should see the visualizer on your computer screen.

### How do I run this simulation (with visualization)?
To run this simulation, open a terminal, change to the drake directory, and
run this example with its default parameters by typing:
```
./bazel-bin/examples/multibody/inclined_plane_with_body/inclined_plane_with_body
```
Note: If the visualizer was launched prior to this step, you should see the
simulation on your computer screen.

### How do I run this simulation using command-line arguments?
To simulate body B as a block that has 4 contacting spheres welded to its
lowest four corners on a 30 degree inclined-plane A (modeled as a half-space), 
pass command line arguments to the executable by typing:
```
./bazel-bin/examples/multibody/inclined_plane_with_body/inclined_plane_with_body -slope_degrees=30 -bodyB_type='block_with_4Spheres'
```

### How do I see a list of all possible command-line arguments?
Open an operating-system prompt, change to the drake directory, and then type:
```
./bazel-bin/examples/multibody/inclined_plane_with_body/inclined_plane_with_body -help
```

### How do I build this simulation (and then run it)?
To build this simulation, open a terminal, change to the drake directory, and
compile/run this example with its default parameters by typing:
```
bazel run examples/multibody/inclined_plane_with_body:inclined_plane_with_body
```
Note: If the visualizer was launched prior to this step, you should see the
simulation on your computer screen.

### How do I build then run this simulation using command-line arguments?
To simulate body B as a block that has 4 contacting spheres welded to its
lowest four corners on a 30 degree inclined-plane A (modeled as a half-space), 
pass command line arguments to the executable by typing:
```
bazel run examples/multibody/inclined_plane_with_body:inclined_plane_with_body -- -slope_degrees=30 -bodyB_type='block_with_4Spheres'
```

### How do I use bazel to see a list of all possible command-line arguments?
To use the bazel build tool to see the list of command-line arguments type:
```
bazel run examples/multibody/inclined_plane_with_body:inclined_plane_with_body -- -help
```

