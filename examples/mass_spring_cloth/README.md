Mass Spring Cloth Example
================================================================================

This example demonstrates the dynamics of a cloth modeled as a mass-spring
system. The cloth, fixed at two corners and initially horizontal, drapes down
and swings under gravity. The purpose of this example is to demonstrate the 
ability of Drake to enable simulation of customized dynamical systems through
user-defined time derivative update for continuous systems and discrete states 
update for discrete systems. Note that this example is a mass-spring system
whose parameters are tuned to exhibit cloth-like behaviors without
considerations of environmental forces or collisions and contact. In particular,
it does not indicate that Drake supports cloth simulation at this point.

The following instructions assume Drake was
[built using bazel](https://drake.mit.edu/bazel.html?highlight=bazel).

All instructions assume that you are launching from the `drake` workspace
directory.
```
cd drake
```

Prerequisites
-------------

Ensure that you have built the drake visualizer with
```
bazel build //tools:drake_visualizer
```

Ensure that you have built the mass spring cloth with
```
bazel build //examples:mass_spring_cloth
```



Mass Spring Cloth Simulation
---------------------

Launch the visualizer
```
bazel-bin/tools/drake_visualizer
```

Launch the mass spring cloth simulation
```
bazel-bin/examples/mass_spring_cloth/mass_spring_cloth
```
