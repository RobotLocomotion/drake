This directory contains a drake-compatible description of the PR2 and an example simulation that simply loads the description, with contact parameters and integrator parameters that are reasonably stable for gripping objects.

The description differs from a standard PR2 description because it uses two prismatic joints and one revolute joint to move along the ground plane (named x, y, and theta, respectively), instead of wheels. The description also contains fully actuated finger joints, instead of a single prismatic joint that closes and opens finger joints that mimic each other.


To build the simulation:
$ bazel build //drake/examples/pr2:pr2_simulation

To run the simulation from the root directory of Drake:
$ ./bazel-bin/drake/examples/pr2/pr2_simulation
