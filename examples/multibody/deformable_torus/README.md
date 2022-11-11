# Deformable torus

This is an example of simulation of deformable bodies in Drake.
The example poses a deformable torus on the ground and uses a PD controlled
gripper that follows a prescribed kinematics to pick up the torus, lift it up in
the air, and then drop it back on the ground.
This demonstrates the dynamics of deformable bodies and showcases the SAP solver
in handling contact between deformable and rigid bodies.

In the source code, this example shows how to set up deformable bodies in a 
Drake simulation and highlights the difference between deformable and rigid
bodies.

## Run visualizer

```
bazel run //tools:meldis -- --open-window &
```

## Run the example

```
bazel run //examples/multibody/deformable_torus:deformable_torus
```

## Options

There are a few command-line options that you can use to adjust the physical
properties of the deformable body. Use `--help` to see the list.

```
bazel run //examples/multibody/deformable_torus:deformable_torus -- --help
```
