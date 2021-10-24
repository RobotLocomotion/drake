IIWA Manipulation Examples
==========================

There are a number of examples contained in these directories.

The following instructions assume Drake was
[built using bazel](https://drake.mit.edu/bazel.html?highlight=bazel).

Prerequisites
-------------

Ensure that you have installed the drake visualizer with
```
bazel build //tools:drake_visualizer
```

All instructions assume that you are launching from the `drake`
workspace directory.
```
cd drake
```


Basic IIWA Simulation
---------------------

Launch the visualizer
```
bazel-bin/tools/drake_visualizer
```

Launch the kuka simulation
```
bazel-bin/examples/kuka_iiwa_arm/kuka_simulation
```

Launch the "plan runner" (which produces position commands over time
upon receiving a single plan message)
```
bazel-bin/examples/kuka_iiwa_arm/kuka_plan_runner
```

Command the robot to move the end effector
```
bazel-bin/examples/kuka_iiwa_arm/move_iiwa_ee -x 0.8 -y 0.3 -z 0.25 -yaw 1.57
```
