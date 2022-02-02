# Planar Gripper Example

This directory contains an example for simulating a planar-gripper
(three two-degree-of-freedom fingers moving in a plane) which reorients a brick
through contact interactions.

## Prerequisites

All instructions assume that you are launching from the `drake`
workspace directory.
```
cd drake
```

Open a visualizer window
```
bazel run //tools:meldis -- --open-window &
```

Build the example in this directory
```
bazel build //examples/planar_gripper/...
```

## Example

### Run Trajectory Publisher

```
bazel-bin/examples/planar_gripper/run_planar_gripper_trajectory_publisher
```

Sends desired joint positions over LCM. Requires a suitable LCM based
simulator (such as the example in this directory) listening for
`lcmt_planar_gripper_command` messages and publishing
`lcmt_planar_gripper_status` messages. If successful, the program will
display  `Waiting for first lcmt_planar_gripper_status`, indicating it is
waiting for the simulation to start publishing status messages.


### Simulation
Run the following example of an (LCM based) simulation of a planar-gripper:


```
bazel-bin/examples/planar_gripper/planar_gripper_simulation
```

This simulates a planar-gripper with a position-based inverse dynamics
controller (by default), communicating via LCM.
