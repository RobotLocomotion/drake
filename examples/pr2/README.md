This directory contains a drake-compatible description of the PR2.

The description differs from a standard PR2 description because it uses two
prismatic joints and one revolute joint to move along the ground plane (named x,
y, and theta, respectively), instead of wheels. The description also contains
fully actuated finger joints, instead of a single prismatic joint that closes
and opens finger joints that mimic each other.

[1] To view or use the C++ passive simulation of a PR2 based on RigidBodyTree
(removed as of 2020), you may use this commit:

https://github.com/RobotLocomotion/drake/tree/v0.14.0/examples/pr2
