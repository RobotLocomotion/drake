Mass Spring Cloth Example
================================================================================

This example demonstrates the dynamics of a cloth modeled as a collection of
particles connected by springs. The cloth, fixed at two corners and
initially horizontal, drapes down and swings under gravity. The purpose of this
example is to demonstrate the ability of Drake to enable simulation of
customized dynamical systems through user-defined time derivative update for
continuous systems and discrete states update for discrete systems. Note that
this example is a particle system connected by springs whose parameters are
tuned to exhibit cloth-like behaviors without considerations of environmental
forces or collisions and contact. In particular, it does not indicate that Drake
supports cloth simulation at this point.

The system runs in discrete mode when the timestep `dt` is set to be positive,
and it runs in continuous mode when the timestep `dt` is set to be zero or
negative. As the discrete and the continuous mode share many common features, we
implement them in a single system to minimize code duplication. Alternatively,
one could implement them as two separate systems.

The mass-spring model is adapted from [Provot 1995]. The discrete time
integration scheme follows that of [Bridson, 2005].

The following instructions assume Drake was
[built using bazel](https://drake.mit.edu/bazel.html?highlight=bazel).

All instructions assume that you are launching from the `drake` workspace
directory.
```
cd drake
```

Prerequisites
-------------

Ensure that you have the visualizer and the demo itself built:

```
bazel build //tools:drake_visualize //examples/mass_spring_cloth:run_cloth_spring_model
```


Mass Spring Cloth Simulation
---------------------
In one terminal, launch the visualizer
```
bazel-bin/tools/drake_visualizer
```

In another terminal, launch the mass spring cloth simulation
```
bazel-bin/examples/mass_spring_cloth/run_cloth_spring_model
```

The discrete mode is run by default. Since the discrete solver uses a
conditionally stable time integration scheme. Using too large a `dt`
may lead to instability. Usually, you will need to decrease `dt` when you:

 1. increase the elastic stiffness (accessible via the parameter file 
 `drake/examples/mass_spring_cloth/cloth_spring_model_params_named_vector.yaml`),
 2. decrease `h`, or
 3. decrease the mass of the particles.

To switch to the continuous mode, add the flag `--dt=0`. The number
of particles in the x-direction and the y-direction can be configured
with the flag `--nx` and `--ny` respectively, and the separation
between particles can be set with the flag `--h`. Use `--help` to
get the full list of flags.

References
-------------------------------------------------
 - [Bridson, 2005] Bridson, Robert, Sebastian
 Marino, and Ronald Fedkiw. "Simulation of clothing with folds and wrinkles."
 ACM SIGGRAPH 2005 Courses. 2005.
 - [Provot 1995] Provot, Xavier.
 "Deformation constraints in a mass-spring model to describe rigid cloth
 behaviour." Graphics interface. Canadian Information Processing Society, 1995.
