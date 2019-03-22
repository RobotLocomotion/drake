# inclined_plane_with_body example

## Description
This simulates the motion of a rigid body B (e.g., a sphere or a block) on an
inclined-plane A (which may be an infinite half-space or a finite box).
For example, it can simulate a uniform-density sphere B rolling down an
inclined-plane A or it can simulate a block B that is sticking or slipping
on an inclined-plane A that is modeled as either an infinite half-space or
as a finite-sized box (so the block can fall off the inclined-plane).


## How do I start the visualizer?
To visualize this example, first open a terminal, change to the drake directory
and type something like the following at the operating system command prompt
(this invokes bazel to build and start the drake_visualizer):
```
bazel run //tools:drake_visualizer
```

Alternately, once the visualizer is built, you can start it by typing:
```
./bazel-bin/tools/drake_visualizer &
```


## How do I build and run this simulation (with visualization)?
To build this simulation, open a terminal, change to the drake directory, and
compile/run this example with its default parameters by typing:
```
bazel run examples/multibody/inclined_plane_with_body:inclined_plane_with_body
```
Note: If the drake_visualizer has launched, you should see the simulation on
your computer terminal.


## How do I run this simulation using command-line arguments?
To simulate body B as a block that has 4 contacting spheres welded to its
lowest four corners on an inclined-plane A (modeled as a half-space), pass
command line arguments to the executable by typing (all on one command line)
```
bazel run examples/multibody/inclined_plane_with_body:inclined_plane_with_body --
 --target_realtime_rate=0.5 --simulation_time=2.8 --time_step=1.0E-4
 --slope_degrees=30 --is_inclined_plane_half_space=true
 --penetration_allowance=1.0E-4 --muS_bodyB=0.1 --muK_bodyB=0.1
 --bodyB_type='block_with_4Spheres'
```


## How do I see a list of command-line arguments?
Open an operating-system prompt, change to the drake directory, and then type:
```
bazel run examples/multibody/inclined_plane_with_body:inclined_plane_with_body -- --help
```