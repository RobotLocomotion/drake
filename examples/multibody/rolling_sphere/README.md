#Rolling Sphere Example

This directory contains an example for illustrating the difference between
contact models. It consists of a rolling sphere on a horizontal ground surface.

```
                                <─── wy
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
corresponding parameters vx and wy, respectively). The ball will begin sliding
in the +Wx direction but eventually friction will cause the ball to slow and
then accelerate in the -Wx direction.

__Contact models__

There are three contact models that can be exercised (via the `--contact_model`
parameter). They differ in how they model the contact between two penetrating
objects:

  - "point": modeled as a point pair.
  - "hydroelastic": _possibly_ modeled as a contact surface.
  - "hybrid": modeled as a contact surface where possible, and as a point.
    pair otherwise. More formally known as "hydroelastic callback with
    fallback".

The "hydroelastic" model doesn't have full support yet. There are some types
of geometries that cannot yet be given a hydroelastic representation and some
types of contact which aren't modeled yet. For example:
  - Drake `HalfSpace` shapes can't be represented at all
  - Drake `Mesh` shapes can only be modeled with _rigid_ hydroelastic
    representation
  - Contact between two rigid or two soft objects aren't supported.

__Changing the configuration__

This example allows you to experiment with the contact models by changing the
configuration in various ways:
  - Add in an optional wall (as shown below). When using "hybrid" or
    "hydroelastic" contact models, the wall will have a _soft_ hydroelastic
    representation.
  - Change the compliance type of the sphere from soft (default) to rigid for
    the "hybrid" and "hydroelastic" models.

```
     ▒▒▒▒▒
     ▒▒▒▒▒
     ▒▒▒▒▒
  w  ▒▒▒▒▒
  a  ▒▒▒▒▒                       <─── wy
  l  ▒▒▒▒▒                       ooooooo
  l  ▒▒▒▒▒                     o         o
     ▒▒▒▒▒                    o     vx    o
     ▒▒▒▒▒                    o     ──>   o
     ▒▒▒▒▒                    o           o
     ▒▒▒▒▒                     o         o
     ▒▒▒▒▒                       ooooooo
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
                                Ground
```
__Figure 2, Adding the wall__: by adding the wall, the ball will initially
behave as described in __Figure 1__ but eventually hit the wall.


The following table enumerates some configuration options and the simulation
outcome. Default values are indicated as "(d)". (Note: the ground is _always_
rigid for the "hydroelastic" and "hybrid" contact models.)

|  Wall  |  Sphere  | Contact Model | Note |
| ------ | -------- | ------------- | ------- |
| no (d) | soft (d) |   point (d)   | Behavior as described in Figure 1 - point pairs visualized |
| no (d) | soft (d) | hydroelastic  | Behavior as described in Figure 1 - contact surfaces visualized |
| no (d) | soft (d) |    hybrid     | Same as "hydroelastic" |
| no (d) |  rigid   |   point (d)   | Behavior as described in Figure 1 - the sphere's rigid declaration is meaningless for point contact and ignored |
| no (d) |  rigid   | hydroelastic  | Throws _immediate_ exception -- cannot support rigid-rigid contact surface between ground and sphere |
| no (d) |  rigid   |    hybrid     | Behavior as described in Figure 1 - rigid-rigid contact (between sphere and ground) uses point-pair contact |
|  yes   | soft (d) |   point (d)   | Behavior as described in Figure 2 - point pairs visualized |
|  yes   | soft (d) | hydroelastic  | Behavior as described in Figure 2 - Throws exception when sphere hits wall; cannot support soft-soft contact |
|  yes   | soft (d) |    hybrid     | Behavior as described in Figure 2 - sphere-ground contact is contact surface, sphere-wall contact is point pair |
|  yes   |  rigid   |    hybrid     | Behavior as described in Figure 2 - sphere-ground contact is point pair, sphere-wall contact is contact surface |


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
```
bazel-bin/examples/multibody/rolling_sphere/rolling_sphere_run_dynamics
```

##### Default behavior with hydroelastic contact
```
bazel-bin/examples/multibody/rolling_sphere/rolling_sphere_run_dynamics --contact_model=hydroelastic
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
