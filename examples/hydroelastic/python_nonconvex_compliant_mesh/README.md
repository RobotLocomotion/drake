# Drop a bell pepper

This is a simple example of using non-convex meshes in the hydroelastic contact
model in Python.
The example drops a compliant-hydroelastic yellow bell pepper on a 
rigid-hydroelastic bowl.
The bell pepper is a non-convex compliant mesh.
The bowl is a non-convex rigid mesh.

## Run Drake Visualizer

```
bazel run //tools:drake_visualizer &
```

In `Plugins > Contacts > Configure Hydroelastic Contact Visualization` you
might want to set these:

* Vector scaling mode = Scaled
* Global scale of all vectors = 0.1

## Run the example

```
bazel run //examples/hydroelastic/python_nonconvex_compliant_mesh:drop_pepper_py
```

## Inspect the initial condition

```
bazel run //examples/hydroelastic/python_nonconvex_compliant_mesh:drop_pepper_py \
-- --simulation_time=0
```

With `--simulation_time=0`, the simulation stops at the beginning, so we can 
see from where we drop the bell pepper.

## Slow down to observe dynamics

```
bazel run //examples/hydroelastic/python_nonconvex_compliant_mesh:drop_pepper_py \
-- --target_realtime_rate=0.1
```

With `--target_realtime_rate=0.1`, the simulation runs at 0.1X realtime rate.
We can observe how the bowl rocks back and forth and how the bell pepper 
turns upside down.

## Inspect the contact surface

After the simulation finishes, in `drake_visualizer`, right-click on the 
bell pepper and set Properties to have Alpha = 0.5.
You can see that the contact surface between the bell pepper and the bowl 
consists of a few patches.
One contact force is applied at the centroid of these patches. 
