<h1>Rolling ball on a dinner plate on a floor</h1>

This is an example for using hydroelastic contact model with a non-convex
collision geometry loaded from an SDFormat file of a dinner plate.
The ball, the plate, and the floor are compliant, rigid, and compliant
hydroelastic. The plate-ball, ball-floor, and plate-floor contacts are 
rigid-compliant, compliant-compliant, and rigid-compliant.
Hydroelastic contact model can work with non-convex shapes accurately
without resorting to their convex hulls.

![ball_plate](images/ball_plate.jpg)

<h2> Preliminary Step: drake_visualizer </h2>

```
bazel run //tools:drake_visualizer &
```

<h2>Run with Hydroelastic</h2>
By default, this example uses hydroelastic contact model.
It is intentionally run at 0.1 realtime rate, so we can appreciate dynamics
in the visualization. Otherwise, the simulation is too fast for human eyes.

```
bazel run //examples/hydroelastic/ball_plate:ball_plate_run_dynamics
```

<h2>Use polygon or triangle contact surfaces</h2>
By default, this example uses polygon contact surfaces that look like this:

![ball_plate](images/ball_plate_contact_polygons.jpg)

The option `--hydro_rep=tri` specifies triangle contact surfaces:
```
bazel run //examples/hydroelastic/ball_plate:ball_plate_run_dynamics \
-- --hydro_rep=tri
```
It will run slower and look like this:

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
the `--simulation_time` option.
Compared to hydroelastic contact, the point contact has to pick one 
representative point from each hydroelastic contact patch.
Since the contact patch between the dinner plate and the floor is quite large,
the chosen contact point keeps oscillating on the patch.
The point contact is hyper realtime, so we slow it down to realtime with the
option `--simulator_target_realtime_rate=1.0` in order to see the animation 
easier.

