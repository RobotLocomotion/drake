# Rolling Sphere Example

This directory contains an example for illustrating the difference between
contact models. It consists of a rolling sphere on a horizontal ground surface.

```
                                <─── ωy
                                ooooooo
                              o         o
                             o     vx    o
                             o     ──>   o
                             o           o
                              o         o
                                ooooooo
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
                                Ground
```
__Figure 1, Default configuration:__ The example creates a sphere with initial
translational velocity in the +Wx direction and an angular velocity around the
-Wy axis (illustrated by the two vectors labeled with the application's
corresponding parameters vx and ωy, respectively). The ball will begin sliding
in the +Wx direction but eventually friction will cause the ball to slow and
then accelerate in the -Wx direction.

Both the ground and the sphere can be modeled as being compliant or rigid
in the hydroelastic model. However, choice of compliance type (more particularly
contact between objects of particular compliance) will constrain your choice
of contact model. See details below.

__Contact models__

There are three contact models that can be exercised (via the `--contact_model`
parameter). They differ in how they model the contact between two penetrating
objects:

  - "point": modeled as a point pair. The compliance type of the objects do
    not matter; contact will always be reported and resolved.
  - "hydroelastic": modeled as a contact surface (where supported). If contact
    between a pair of objects is detected, but hydroelastic cannot evaluate a
    contact surface between them, the simulation will throw. See below for
    circumstances under which a contact surface cannot be computed.
  - "hybrid": modeled as a contact surface where possible, and as a point.
    pair otherwise. More formally known as "hydroelastic callback with
    fallback".

The "hydroelastic" model doesn't support all combinations of geometries and
compliance types. There are some types of geometries that cannot yet be given a
hydroelastic representation and some types of contact which aren't modeled yet.

  - Contact between a rigid shape and compliant shape is supported.
  - Contact between two rigid or two compliant objects isn't supported.
  - Drake `Mesh` shapes can only be modeled with _rigid_ hydroleastic
    representation.

__Solvers__

When the system is modeled as continuous, the multibody
system's dynamics is described by an ODE of the form `ẋ = f(t,x)`. Drake's
simulator then uses error-controlled integration to advance this dynamics
forward in time. When the system is modeled as discrete, the multibody system's
dynamics is described by an algebraic equation of the form
`x[n+1] = f(t[n],x[n])`. This algebraic relation involves a complex
computation performed by Drake's contact solver which solves the system's
dynamics subject to contact constraints.

This example allows us to choose between a continuous or discrete modeling of
the system's dynamics for both point and hydroelastic contact. To enable the
discrete model using an update period of one millisecond, supply the additional
command line flag `--mbp_dt=1.0e-3`. The continuous approximation is the default
(with `--mbp_dt=0`).

__Changing the configuration__

This example allows you to experiment with the contact models by changing the
configuration in various ways:

  - Add in an optional compliant wall (as shown below). When using "hybrid" or
    "hydroelastic" contact models, the wall will have a _soft_ hydroelastic
    representation.
  - Change the compliance type of the sphere from compliant (default) to 
    rigid for the "hybrid" and "hydroelastic" models.
  - Change the compliance type of the ground from rigid (default) to 
    compliant for the "hybrid" and "hydroelastic" models.

```
     ▒▒▒▒▒
     ▒▒▒▒▒
     ▒▒▒▒▒
  w  ▒▒▒▒▒                                                Wz
  a  ▒▒▒▒▒                       <─── ωy
  l  ▒▒▒▒▒                       ooooooo                  │   Wy
  l  ▒▒▒▒▒                     o         o                │  ╱
     ▒▒▒▒▒                    o     vx    o               │ ╱
     ▒▒▒▒▒                    o     ──>   o               │╱
     ▒▒▒▒▒                    o           o               └───────  Wx
     ▒▒▒▒▒                     o         o
     ▒▒▒▒▒                       ooooooo
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
                                Ground
```
__Figure 2, Adding the wall__: by adding the wall, the ball will initially
behave as described in __Figure 1__ but eventually hit the wall.

The following table enumerates some configuration options and the simulation
outcome. Default values are indicated as "(d)".

|  Ground   |  Compliant Wall  |     Sphere    | Contact Model | Note |
| --------- | ---------------- | ------------- | ------------- | ---------------------------------- |
| rigid (d) |      no (d)      | compliant (d) |   point (d)   | Behavior as described in Figure 1 - point pairs visualized |
| rigid (d) |      no (d)      | compliant (d) | hydroelastic  | Behavior as described in Figure 1 - contact surfaces visualized |
| rigid (d) |      no (d)      | compliant (d) |    hybrid     | Same as "hydroelastic" |
| rigid (d) |      no (d)      |  rigid        |   point (d)   | Behavior as described in Figure 1 - the sphere's rigid declaration is meaningless for point contact and ignored |
| rigid (d) |      no (d)      |  rigid        | hydroelastic  | Throws _immediate_ exception -- cannot support rigid-rigid contact surface between ground and sphere |
| rigid (d) |      no (d)      |  rigid        |    hybrid     | Behavior as described in Figure 1 - rigid-rigid contact (between sphere and ground) uses point-pair contact |
| rigid (d) |       yes        | compliant (d) |   point (d)   | Behavior as described in Figure 2 - point pairs visualized |
| rigid (d) |       yes        | compliant (d) | hydroelastic  | Behavior as described in Figure 2 - Throws exception when sphere hits wall; cannot support compliant-compliant contact |
| rigid (d) |       yes        | compliant (d) |    hybrid     | Behavior as described in Figure 2 - sphere-ground contact is contact surface, sphere-wall contact is point pair |
| rigid (d) |       yes        |  rigid        |    hybrid     | Behavior as described in Figure 2 - sphere-ground contact is point pair, sphere-wall contact is contact surface |
| compliant |     either       | compliant (d) | point/hybrid  | Compliant-compliant contact requires "hybrid" or "point" -- crashes with "hydroelastic" |  
| compliant |     either       |  rigid        |     any       | Behavior as described in Figure 2 - contact visualized depends on model |  

## Prerequisites

All instructions assume that you are launching from the `drake`
workspace directory.
```
cd drake
```

Ensure that you have built the Drake visualizer with
```
bazel build //tools:drake_visualizer
```

Build the example in this directory
```
bazel build //examples/multibody/rolling_sphere/...
```

## Visualizer

Before running the example, launch the visualizer (building it as necessary):
```
bazel-bin/tools/drake_visualizer
```

## Example

Various incarnations of the example combining the parameters in ways to
illustrate the differences that contact model and hydroelastic representation
can make.

##### Default behavior
The command below runs the default configuration using a continuous modeling of
the dynamics.
```
bazel-bin/examples/multibody/rolling_sphere/rolling_sphere_run_dynamics
```

##### Default behavior, discrete solver
Run the same configuration as above, but this time using a discrete
approximation of the dynamics with an update period of one millisecond.
```
bazel-bin/examples/multibody/rolling_sphere/rolling_sphere_run_dynamics --mbp_dt=1.0e-3
```
The discrete solver supports all other configurations below, including hybrid
contact.

##### Default behavior with hydroelastic contact with default compliance type; rigid ground, compliant ball
```
bazel-bin/examples/multibody/rolling_sphere/rolling_sphere_run_dynamics --contact_model=hydroelastic
```

##### Default behavior with hydroelastic contact with reversed compliance; compliant ground, rigid ball
```
bazel-bin/examples/multibody/rolling_sphere/rolling_sphere_run_dynamics --contact_model=hydroelastic --rigid_ball=1 --soft_ground=1
```

##### Default behavior with hybrid contact
```
bazel-bin/examples/multibody/rolling_sphere/rolling_sphere_run_dynamics --contact_model=hybrid
```

##### Add the wall with hydroelastic contact -- throw when sphere hits the wall
```
bazel-bin/examples/multibody/rolling_sphere/rolling_sphere_run_dynamics --contact_model=hydroelastic --add_wall=1
```

##### Add the wall with hybrid contact
```
bazel-bin/examples/multibody/rolling_sphere/rolling_sphere_run_dynamics --contact_model=hybrid --add_wall=1
```

##### Add the wall, make the sphere rigid with hybrid contact
```
bazel-bin/examples/multibody/rolling_sphere/rolling_sphere_run_dynamics --contact_model=hybrid --add_wall=1 --rigid_ball=1
```
