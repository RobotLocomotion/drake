This directory contains a drake-compatible description of the PR2 and an example
uncontrolled simulation that simply loads the description, with contact
parameters and integrator parameters that are reasonably stable for gripping
objects.

The description differs from a standard PR2 description because it uses two
prismatic joints and one revolute joint to move along the ground plane (named x,
y, and theta, respectively), instead of wheels. The description also contains
fully actuated finger joints, instead of a single prismatic joint that closes
and opens finger joints that mimic each other.


To see the simulation:

1. Open a terminal and navigate to the root of the Drake distribution. Then,
build and launch the drake_visualizer, with the following commands: 
$ bazel build //tools:drake_visualizer 
$ ./bazel-bin/tools/drake_visualizer

2. Open another terminal and navigate to the root of the Drake distribution.
Then, build and launch the pr2_passive_simulation, with the following commands:
$ bazel build //examples/pr2:pr2_passive_simulation 
$ ./bazel-bin/examples/pr2/pr2_passive_simulation 
The seconds (relative to the simulation) that the simulation will run for can 
be specified with the argument --simulation_sec=<seconds to simulate>. If this
argument is not specified, then the simulation will run forever.
