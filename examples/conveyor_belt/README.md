# Conveyor belt

This example demonstrates MultibodyPlant's *surface velocity* feature: bodies
whose surfaces move relative to the body itself, such as conveyor belts,
spinning drums, or tank treads.

The scene is a closed, square conveyor loop built from four belt segments, each
welded to the world, with a deformable ball resting on each segment. Although
the belts never move -- they are rigidly welded and their configuration is
constant for the whole simulation -- the balls are carried around the loop,
handing off from one segment to the next at each corner.

That is the point of the demonstration. Surface velocity is purely a *contact*
effect: it modifies the relative velocity at contact points when computing
contact forces, and plays no role in any other MultibodyPlant computation. No
joint moves, no body is actuated, and the multibody dynamics are otherwise
untouched. See the "Surface velocity" section (`@ref mbp_surface_velocity`) of
the MultibodyPlant documentation for the full model.

Because the belts are geometrically static, the visualizer conveys their motion
by scrolling the belt texture along the surface. That scroll is driven by the
plant's `surface_displacements` output port (the integral of the commanded
speed), which `AddDefaultVisualization()` connects to MeshcatVisualizer
automatically for any body that declares a surface velocity axis.

## What the example illustrates

Declaring a surface velocity in SDFormat (`conveyor_belt.sdf`):

```xml
<link name="belt">
  <drake:surface_velocity_axis>0.0 -1.0 0.0</drake:surface_velocity_axis>
  ...
</link>
```

The axis is the *rotation* axis of the equivalent drum, expressed in the body
frame B. At a contact point the surface velocity is

    v_ss_B = speed * (axis_B x n_B)

where `n_B` is the contact normal pointing out of the body's geometry. For a
belt lying flat, an axis of `-y` and a top-face normal of `+z` give motion
along `-x`. Rotating a segment about the world z-axis therefore aims its
transport direction, which is how the four segments are arranged into a
circulating loop in `conveyor_example.dmd.yaml`.

Commanding the speed at runtime (`conveyor_belt.cc`): the axis only fixes the
*direction*; the signed speed comes from the plant's `surface_speeds` input
port, a bus whose signals are keyed by scoped body name. An unconnected port
means zero speed, so nothing would move. The example iterates over the plant's
bodies, and for each one that has a registered axis, adds a signal to the bus:

```c++
systems::BusValue speeds;
for (multibody::BodyIndex i(0); i < plant.num_bodies(); ++i) {
  const multibody::RigidBody<double>& body = plant.get_body(i);
  if (plant.GetSurfaceVelocityAxis(body).has_value()) {
    speeds.Set(body.scoped_name().to_string(), Value<double>(FLAGS_belt_speed));
  }
}
```

Because the speed is an input port rather than a fixed property, it can be
driven by any system -- a controller, a trajectory, or, as here, a
`ConstantValueSource`.

## Run the example

```
bazel run //examples/conveyor_belt:conveyor_belt
```

The program will print a Meshcat URL to the console to watch the simulation.
Prefer using that URL over a M session -- Meldis will show the motion of the
deformable spheres, but won't be able to visualize the movement of the surfaces.

## Options

| Flag | Default | Meaning |
| --- | --- | --- |
| `--belt_speed` | 0.5 | Surface speed (m/s) commanded for every body that declares a surface velocity axis. Negative values reverse the loop. |
| `--simulation_time` | 100 | Duration of the simulation in seconds. |
| `--realtime_rate` | 1.0 | Target realtime rate; use 0 to run as fast as possible. |
| `--max_time_step` | 1e-2 | Discrete time step of the plant. |

Use `--help` for the full list.

```
bazel run //examples/conveyor_belt:conveyor_belt -- --belt_speed=-0.5
```

Setting `--belt_speed=0` is instructive: the scene is then completely inert,
which shows that everything the balls do comes from the surface velocity alone.
