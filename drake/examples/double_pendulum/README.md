# Double pendulum demo

## Description

This is a demo that spawns a double pendulum from an SDF description using the sdformat library. Both
simulation time and realtime rate are exposed as command line arguments.

```
    Rigid Body Tree
           +
           |
           |
+----------+----------+                +--------------------+
|                     |                |                    |
|                     |                |                    |
|   Rigid Body Plant  +--------------->+  Drake Visualizer  |
|                     |                |                    |
|                     |                |                    |
+---------------------+                +--------------------+

```

## How do I build it?

To build this demo, from Drake's repository root path just run:

```
bazel build drake/examples/double_pendulum:double_pendulum_demo
```

## How do I run it?

To run this demo, from Drake's repository root just run:

```
bazel run drake/examples/double_pendulum:double_pendulum_demo -- [-simulation_time +inf] [-realtime_rate 1.0]
```
