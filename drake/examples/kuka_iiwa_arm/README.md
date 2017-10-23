IIWA Manipulation Examples
==========================

There are a number of examples contained in these directories.  Many of them require launching multiple processes.  To aid in this process, we use "procman" -- a simple tool for managing processes on multiple machines via LCM.

The following instructions assume Drake was
[built using bazel](http://drake.mit.edu/bazel.html?highlight=bazel).

Prerequisites
-------------

Ensure that you have set your [PYTHONPATH](http://drake.mit.edu/python_bindings.html?highlight=python).


Basic IIWA Simulation
---------------------

```
$ cd drake-distro
$ ./build/install/bin/bot-procman-sheriff -l drake/examples/kuka_iiwa_arm/kuka_sim.pmd &
```

This will open a menu of processes that you can run.
Recommended workflow:
* Start the drake-visualizer (in the tools group).  This must be started before a simulation to visualize the robot.
* Start the entire "sim" group.  This will launch the dynamics backend, with a basic controller and plan playback.
* Start the director IK interface (in the tools group), and try piloting the robot with a few clicks in the IK panel, followed by pressing the PLAN button to see a preview and then the EXECUTE button to see the simulated robot actually execute the plan.
