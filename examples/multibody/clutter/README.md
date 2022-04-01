# Clutter

To run the default configuration you can simply do (don't forget to start the
visualizer before running the simulation):
```
bazel run examples/multibody/mp_convex_solver:clutter
```

To run with specific options, compile and run with:
```
bazel run -j 4 --config gurobi examples/multibody/mp_convex_solver:clutter -- --mbp_time_step=1e-2 --objects_per_pile=10 --simulator_target_realtime_rate=0 --simulation_time=4.0
```

The most relevant options are:
- `mpb_time_step`: specifies the discrete solver time step.
- `objects_per_pile`: an arbitrary assortment of objects are initialized into
  four piles, each having `objects_per_pile` objects.
- `simulation_time`: The duration of the simulation in seconds.
- `visualize_forces`: Set to `true` to enable the visualization of contact
  forces.

## Experimental contact solver

This demo allows (enabled by default) to run the SAP contact solver.

If you want to use this solver in your own Drake demos you'll need:

1. Include `"drake/multibody/plant/compliant_contact_computation_manager.h"`.

2. In your `BUILD.bazel` add dependency to
   `"//multibody/plant:compliant_contact_computation_manager"`.

3. After `MultibodyPlant::Finalize()` make the call:
```c++
plant.SetDiscreteUpdateManager(std::make_unique<CompliantContactComputationManager<double>>());
```

Notice that this clutter demo in addition sets non-default parameters for the
solver. This is not required and the default parameters should work for your
application.

### Contact parameters

The compliance parameters for both point contact and hydroelastics do not
change. However, this solver uses a linear model of dissipation which introduces
a separate contact parameter called `dissipation_time_constant`, the Hunt &
Crossley dissipation used in Drake by default is ignored. Therefore be aware
that changes to dissipation through SDF and/or URDF files will not have any
effect when using the new contact solver this branch.

Think of the `dissipation_time_constant` as the time, in seconds, it takes to
the compliant material to relax from a deformed configuration to its rest
configuration. For instance, poke your finger with a pen. The time it takes your
skin to go back to its undeformed rest state is a good proxy of the
`dissipation_time_constant`. A good starting point for this parameter (the
default) is to set it to the `MultibodyPlant` time step. This leads to a good
approximation of inelastic collisions, which is what we most often want for
robotics applications.

For completeness, this demo sets all contact parameters programatically by
changing the default `ProximityProperties` or each contact geometry. Take a look
at the source for an example on how to do this in your own sims.

## Large number of contacts per box.
Run this case with:
```
bazel run examples/multibody/mp_convex_solver:clutter  -- --simulation_time=1 --visualize=0 --num_spheres=10
```
Visualization is disabled for more stable time measurements. Enable it with
`visualize=1` if desired.

This example emulates multicontact by placing an array of spheres along each
face of the boxes. The idea is to be able to test the scalability of solver with
the number of contact points independent of the number of bodies.

The flag `num_spheres` controls the number of spheres per face. We place an
array of size `num_spheres x num_spheres` of spheres on each face.

**Note:** To avoid the typical rattle caused by the unstable box-box point
contact (jumping from corner to corner when two flat faces meet), this case
*filters out collision between boxes*. Only box-sphere collision is allowed. If
desired, box-box contact can be enabled by setting `enable_box_box_collision=1`.

## Solution File

(WARNING): Only relevant to developers.

This test case produces two solution files:
- `log.dat`: Records solver statistics, such as number of contacts, errors
  relative to the analytical inverse dynamics, and wall clock for different
  phases of the solver.
- `manager_log.dat`: Records timing statistics at the level of
  `CompliantContactComputationManager`.  

Within the Bazel environment these files will be found at
`bazel-bin/examples/multibody/mp_convex_solver/clutter.runfiles/drake`.

**Note:** The file `manager_log.dat` is an ASCII file with each column
containing, in this order:
 - time, num_contacts 
 - total_time: The "ContactManager" total time. This should match the total sim
   time modulo system framework overhead.
 - geometry_time: The time spend in `EvalFoo()` geometry queries. In the
   abscence of any other publish events, this amounts to the time spend in the
   actual `CalcFoo()` method.
 - vstar_time: Computation of velocities "before contact". -graph_time: Sparsity
   analysis. -linear_dynamics_time: Sparse linearized dynamics.
   -contact_jacobian_time: Sparse contact Jacobian. -contact_solver_time: the
   actual contact solver time. -pack_results_time: packing contact results for
   reporting.
