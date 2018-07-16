Simple Gripper Simulation
=========================

This simple gripper demo is used to exercise MultibodyPlant's contact modeling
in a gripping scenario. SceneGraph is used for both visualization and contact
handling.

This example models a gripper with two fingers. The right finger is fixed to the
main body of the gripper, while the left finger has a prismatic joint to allow
opening and closing the grip. The grip force can be specified with the command
line option `--gripper_force`, which defaults to 10 Newtons of force. If we
provide a value of exactly zero, the fingers of the gripper will be set to be at
`--grip_width` distance (in meters) from each other. When `--gripper_force > 0`,
`--grip_width` specifies the initial condition for the fingers position.

The demo also permits specifying a vertical harmonic motion of the gripper to
evaluate the performance of the grip when the gripper is undergoing forced
motions. The parameters controlling this motion are `--amplitude`, to specify
the amplitude of the oscillations in meters and, `--frequency` to specify the
frequency of the oscillations in Hertz. Additionally, the user might like to
change the direction of the forced oscillations to either be in the horizontal
or vertical direction. This can be accomplished by changing the `<axis>`
specified for the `<joint>` named "translate_joint" in the SDF for the gripper
model, `simple_gripper.sdf`. Only vertical or horizontal forced motions are
supported in this example and thus the axis must either be the x-axis or the
z-axis. Otherwise an exception is thrown communicating this fact. There is no
gravity for the forced vertical oscillations case.

Fixed Gripper with Gravity
--------------------------
To run a case with gravity but no forced oscillations, set the `<joint>` axis
for the joint named "translate_joint" to be the x-axis in the SDF file.
The parameter `--gripper_force` specifies the grip force while `--grip_width`
specifies the initial condition for the position of the fingers.
If the gripper force is set to zero, the fingers will be placed at a fixed
`--grip_width` distance apart. The latter is an important test case since it
eliminates the additional complexity introduced by having moving fingers.

Prerequisites
-------------

From your `drake` workspace directory you first need to build this example and
the drake visualizer.

Ensure that you have built drake visualizer with
```
bazel build //tools:drake_visualizer
```

Build this example with
```
bazel build //examples/simple_gripper
```

Running the Example
-------------------

Launch the visualizer (optionally visualizing contact forces or not)

Without contact forces visualized:
```
./bazel-bin/tools/drake_visualizer```
With contact forces visualized:
```
./bazel-bin/tools/drake_visualizer --script multibody/rigid_body_plant/visualization/contact_viz.py
```

Launch the simulation with
```
./bazel-bin/examples/simple_gripper/simple_gripper --simulation_time=10.0
```
where for this particular invocation example we are specifying the simulation
time in seconds as a command line option. To get a list of command line options
for this example run
```
./bazel-bin/examples/simple_gripper/simple_gripper --help
```
which will also provide a description for each option.
