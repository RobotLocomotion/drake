# Particle examples

## Uniformly accelerated particle sample

### Description

This is a demo that creates a very simple system of a 1-DOF particle, moving at a constant acceleration.  All relevant quantities can be set through the command line: its initial position and velocity, as well its acceleration (in SI units). Additionally, simulation time and realtime rate are also exposed as command line arguments.

### How do I build it?

To build this demo, from Drake's repository root path just run:

```
bazel build examples/particles/uniformly_accelerated_particle_demo
```

### How do I run it?

To run this demo, from Drake's repository root just run:

```
bazel run examples/particles:uniformly_accelerated_particle_demo -- [-initial_position 0.0] [-initial_velocity 0.0] [-acceleration 1.0] [-simulation_time +inf] [-realtime_rate 1.0]
```
