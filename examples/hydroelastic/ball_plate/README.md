<h1>Rolling ball on a dinner plate on a floor</h1>

This is an example for using hydroelastic contact model with a non-convex
geometry loaded from an SDFormat file of a dinner plate.
The ball, the plate, and the floor are compliant, rigid, and compliant
hydroelastic. The plate-ball, ball-floor, and plate-floor contacts are 
rigid-compliant, compliant-compliant, and rigid-compliant.
Hydroelastic contact model can work with non-convex shapes accurately
without resorting to their convex hulls.

In the source code, this example shows how to set up bodies by loading SDFormat
files and also calling C++ APIs.

![ball_plate](images/ball_plate.jpg)

<h2> Preliminary Step: drake_visualizer </h2>

```
bazel run //tools:drake_visualizer &
```

In `Plugins > Contacts > Configure Hydroelastic Contact Visualization` you
might want to set these:

- Maximum pressure = 1e4
- Vector scaling mode = Scaled
- Global scale of all vectors = 0.05

as shown here:

![ball_plate](images/ball_plate_hydroelastic_contact_visualization_settings.jpg)

<h2>Run with Hydroelastic</h2>
By default, this example uses hydroelastic contact model.
It is intentionally run at 0.1 realtime rate, so we can appreciate dynamics
in the visualization. Otherwise, the simulation is too fast for human eyes.

```
bazel run //examples/hydroelastic/ball_plate:ball_plate_run_dynamics
```

If it's too fast to see, you can slow it down further like this:
```
bazel run //examples/hydroelastic/ball_plate:ball_plate_run_dynamics \
-- --simulator_target_realtime_rate=0.01
```

<h2>Use polygon or triangle contact surfaces</h2>
By default, this example uses polygon contact surfaces that look like this:

![ball_plate](images/ball_plate_contact_polygons.jpg)

The option `--contact_surface_representation=triangle` specifies triangle
contact surfaces:
```
bazel run //examples/hydroelastic/ball_plate:ball_plate_run_dynamics \
-- --simulator_target_realtime_rate=0.01 \
--contact_surface_representation=triangle
```
It will look like this:

![ball_plate](images/ball_plate_contact_triangles.jpg)

<h2>Run with point contact model</h2>

```
bazel run //examples/hydroelastic/ball_plate:ball_plate_run_dynamics \
-- --contact_model=point --simulation_time=120.0 \
--simulator_target_realtime_rate=1.0
```

The option `--contact_model=point` selects the point contact model.
Notice the unphysical oscillation of the dinner plate that does not
dissipate energy despite the very long simulated time specified by
`--simulation_time=120.0`.

Compared to hydroelastic contact, the point contact has to pick one
point from each contact patch.
Since the contact patch between the dinner plate and the floor is quite large,
the chosen point keeps oscillating on the patch.
The option `--simulator_target_realtime_rate=1.0` slows it down enough for
human eyes to see.

<h2>Other Options</h2>
There are other command-line options that you can use. Use `--help` to see
the list. For example, you can set Hunt & Crossley dissipation of the ball,
friction coefficient of the ball, initial position and velocity of the ball,
etc.

```
bazel run //examples/hydroelastic/ball_plate:ball_plate_run_dynamics \
-- --help
```
