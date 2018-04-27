# Double pendulum demo

## Description

This is a demo that spawns a double pendulum from an SDF description using the
sdformat library. Both simulation time and realtime rate are exposed as command
line arguments.

```
                parsed with
+-------------+   sdformat  +-----------------+
|             |     into    |                 |
|  SDF model  +------------>+  RigidBodyTree  |
|             |             |                 |
+-------------+             +--------+--------+
                                     |
                   owned by          |
                +--------------------+
                |
                v
      +---------+--------+ rendered +-------------------+
      |                  |    by    |                   |
      |  RigidBodyPlant  +--------->+  DrakeVisualizer  |
      |                  |          |                   |
      +------------------+          +-------------------+

```

## How do I build it?

To build this demo, from Drake's repository root path just run:

```
bazel build examples/double_pendulum:double_pendulum_demo
```

## How do I run it?

To run this demo, from Drake's repository root first run:

```
./bazel-bin/tools/drake_visualizer
```

This will open up the visualization tool. Then run the demo with:

```
bazel run examples/double_pendulum:double_pendulum_demo -- [-simulation_time +inf] [-realtime_rate 1.0]
```
