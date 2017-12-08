IIWA Manipulation Examples
==========================

There are a number of examples contained in these directories.  

The following instructions assume Drake was
[built using bazel](http://drake.mit.edu/bazel.html?highlight=bazel).

Prerequisites
-------------

Ensure that you have installed the drake visualizer with
```
bazel build //tools:drake_visualizer
```

Ensure that you have set your
[PYTHONPATH](http://drake.mit.edu/python_bindings.html?highlight=python).

All instructions assume that you are launching from the `drake-distro`
directory.
```
cd drake-distro
```


Basic IIWA Simulation
---------------------

Launch the visualizer
```
bazel-bin/tools/drake_visualizer
```

Launch the kuka simulation
```
bazel-bin/drake/examples/kuka_iiwa_arm/kuka_simulation
```

Launch the "plan runner" (which produces position commands over time
upon receiving a single plan message)
```
bazel-bin/drake/examples/kuka_iiwa_arm/kuka_plan_runner
```

Coming back soon - generate plans using the graphical IK interface.
See https://github.com/RobotLocomotion/drake/issues/7321 .


