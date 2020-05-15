# Jaco Arm Examples

This directory contains several examples of drake simulating or
interacting with a Kinova Jaco arm.

## Simulation

### Prerequisites

All instructions assume that you are launching from the `drake`
workspace directory.
```
cd drake
```

Ensure that you have installed the drake visualizer with
```
bazel build //tools:drake_visualizer
```

Build the examples in this directory:
```
bazel build //examples/kinova_jaco_arm/...
```

### Examples

Before running any examples, launch the visualizer:
```
bazel-bin/tools/drake_visualizer
```

The following examples of a simulated jaco are present:


```
bazel-bin/examples/kinova_jaco_arm/jaco_simulation
```

Simulates a Jaco arm with an inverse dynamics controller,
communicating via LCM.


## Control

```
bazel-bin/examples/kinova_jaco_arm/jaco_controller
```

Controls a Jaco arm over LCM using joint velocities.  Requires a
suitable LCM based simulator (such as the example in this directory)
or driver listening for ```lcmt_jaco_command``` messages and
publishing ```lcmt_jaco_status```, along with a planner sending robot
plan messages.

```
bazel-bin/examples/kinova_jaco_arm/move_jaco_ee
```

Calculates a motion plan for the Jaco arm, to the end effector
position specified on the command line (with a reachable default
position for testing) and sends the resulting plan over LCM.  Requires
`jaco_controller` to execute the resulting plan.
