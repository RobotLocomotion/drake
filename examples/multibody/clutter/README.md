# Clutter

## Integrators benchmarks

Contact parameters for a non-stiff baseline case: spheres are pretty compliant,
friction coefficient below 1 and stiction tolerance way too large for real
applications. Still, this case is useful for a quick enough baseline to test
integrators.

```bazel run //examples/multibody/clutter:clutter --
--simulator_target_realtime_rate=0.  --mbp_time_step=0.0
--sphere_stiffness=1e3 --friction_coefficient=0.5 --simulator_accuracy=0.01
--stiction_tolerance=0.05 --simulator_integration_scheme=implicit_euler
--simulation_time=3 --objects_per_pile=4```



