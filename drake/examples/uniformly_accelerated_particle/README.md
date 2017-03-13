# Uniformly accelerated particle sample

## Description

This is a sample project that creates a very simple system of a 1-DOF particle, moving at a constant acceleration (see diagram in docs/).  All relvant quantities can be set through the command line: its initial position and velocity, as well its acceleration (in SI units). Additionally, simulation time and realtime rate are also exposed as command line arguments.

## How do I build it?

To build this sample, from Drake's repository root path just run:

```
bazel build drake/examples/uniformly_accelerated_particle:demo
```

## How do I run it?

To run this sample, from Drake's repository root just run:

```
bazel run drake/examples/uniformly_accelerated_particle:demo -- [-initial_position 0.0] [-initial_velocity 0.0] [-acceleration 1.0] [-simulation_time +inf] [-realtime_rate 1.0]
```
