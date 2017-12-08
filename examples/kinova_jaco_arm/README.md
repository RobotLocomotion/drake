# Jaco Arm Examples

This directory contains several examples of drake simulating or
interacting with a Kinova Jaco arm.

## Simulation

### Prerequisites

All instructions assume that you are launching from the `drake-distro`
directory.
```
cd drake-distro
```

Ensure that you have installed the drake visualizer with
```
bazel build //tools:drake_visualizer
```

Build the examples in this directory:
```
bazel build //drake/examples/kinova_jaco_arm/...
```

### Examples

Before running any examples, launch the visualizer:
```
bazel-bin/tools/drake_visualizer
```

The following examples of a simulated jaco are present:

```
bazel-bin/drake/examples/kinova_jaco_arm/run_passive_jaco_demo
```

Simulates a Jaco arm with no control or gravity compensation.

```
bazel-bin/drake/examples/kinova_jaco_arm/run_setpose_jaco_demo
```

Simulates a Jaco arm holding a set pose.

```
bazel-bin/drake/examples/kinova_jaco_arm/run_controlled_jaco_demo
```

Demonstrates planning a trajectory for a Jaco arm using inverse
kinematics and simulating an arm following that trajectory.

## Control

```
bazel-bin/drake/examples/kinova_jaco_arm/jaco_controller
```

Controls a Jaco arm over LCM using joint velocities.  Requires a
suitable LCM based controller listening for ```lcmt_jaco_command```
messages and publishing ```lcmt_jaco_status```.
