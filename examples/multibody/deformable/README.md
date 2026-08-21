# Deformable torus

This is an example of a basic deformable body simulation in Drake.
The example poses a deformable torus on the ground and uses a PD controlled
gripper that follows a prescribed kinematics to pick up the torus, lift it up
in the air, and then drop it back on the ground. Users can switch between a
parallel jaw gripper model and a suction cup gripper model with the `--gripper`
flag.

This demonstrates the dynamics of deformable bodies and showcases the SAP solver
in handling contact between deformable and rigid bodies.

In the source code, this example shows how to set up deformable bodies in a 
Drake simulation and highlights the difference between deformable and rigid
bodies.

# Bubble gripper

This example simulates the [bubble gripper designed at Toyota Research Institute (TRI)](https://www.tri.global/news/sensing-believing-more-capable-robot-hands-soft-bubble-gripper) 
using deformable bodies. This example is also described in the paper
[A Convex Formulation of Frictional Contact between Rigid and Deformable Bodies](https://arxiv.org/abs/2303.08912).

The example poses a deformable teddy bear on the ground and uses PD control to
pick it up with the bubble gripper. A camera is mounted inside the bubble
gripper to show how a dense dot pattern inside the latex membrane is moving and
distorting during the grasp.

This example demonstrates the following capabilities of deformable body
simulation in Drake:
  1. frictional contact resolution among deformable bodies;
  2. deformable geometry rendering;
  3. fixed constraints between rigid bodies and deformable bodies.

In the source code, this example shows how to set up use these functionalities
by calling C++ APIs.

Note that currently (April 2024), this example doesn't run on macOS. We are
working on supporting this on all platforms supported by Drake.

# Deformable disabling

This example showcases the behavior of enabled/disabled deformable models.
Deformable torus bodies are initially placed in a vertical column and disabled.
One by one, they are enabled and fall into contact to demonstrate deformable
behavior. Finally, one of the torus bodies is again disabled to demonstrate that
it is removed from contact participation, causing the other tori to pass through
it.

# Deformable subdivision

This example showcases the ability to subdivide FEM elements to increase the
accuracy of the quadrature rule used to integrate the effect of external
volumetric forces. A coarse deformable torus is added to the scene and a very
concentrated force field acts on it. At zero or low subdivision counts, the
force field falls between the element vertices and has no effect. As the
subdivision count increases, the simulation accuracy improves.

Tune the `--element_subdivision_count` parameter between 0-4 to observe the
effects of greater subdivision counts. Observe that a coarse mesh does not
interact with the concentrated force field, but a subdivided one does.

The source code demonstrates how to assign an element subdivision count.

## Run visualizer

```
bazel run //tools:meldis -- --open-window &
```

## Run the example

```
bazel run //examples/multibody/deformable:deformable_torus
```

or

```
bazel run //examples/multibody/deformable:bubble_gripper
```

or

```
bazel run //examples/multibody/deformable:deformable_disabling
```

## Options

There are a few command-line options that you can use to adjust the physical
properties of the deformable body. Use `--help` to see the list.

```
bazel run //examples/multibody/deformable:deformable_torus -- --help
```

or

```
bazel run //examples/multibody/deformable:bubble_gripper -- --help
```

or

```
bazel run //examples/multibody/deformable:deformable_disabling -- --help
```
