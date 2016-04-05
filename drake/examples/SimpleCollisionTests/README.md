How to run the bouncing ball simulation
=======================================

To get usage info simply run the program without arguments:

```
cd drake-distro/drake/examples/SimpleCollisionTests
../../pod-build/bin/bouncingBall
```

To run a ball bouncing on a flat inclined plane:

```
cd drake-distro/drake/examples/SimpleCollisionTests
../../../build/bin/drake-visualizer &
../../pod-build/bin/bouncingBall ./ball_world.urdf -terrain flat -angle 45
```

The option `-angle` allows to specify the slope of the inclined plane, in degrees.

To run a ball bouncing on a sinusoidal shape height map:
```
cd drake-distro/drake/examples/SimpleCollisionTests
../../../build/bin/drake-visualizer &
../../pod-build/bin/bouncingBall ./ball_world.urdf -terrain height_map -x -1
```

The `-x` option allows to specify a translation of the height map terrain in the x direction. The ball has a diameter of 1.0. Therefore `-x 1` shifts the height map position one sphere diameter in the positive x direction.

Unit Test: Bouncing ball on 45 degrees inclined plane
=====================================================

This test corresponds to running the bouncing ball simulation as:

```
cd drake-distro/drake/examples/SimpleCollisionTests
../../../build/bin/drake-visualizer &
../../pod-build/bin/bouncingBall ./ball_world.urdf -terrain flat -angle 45
```

However the Google test is run as:

```
cd drake-distro/drake/examples/SimpleCollisionTests
../../pod-build/bin/simple_collisions_gtest
```

This test will compute the solution using a time step `delt=1.0e-3` and a penetration stiffness of `penetration_stiffness=50000.0`.
An analytical solution for this problem is described in `simple_collisions_gtest.cc`.
However, the numerical solution does not converge to the analytical solution as the time step is reduced. The reason for this is that
the numerical solution is computed with a penetration stiffness while the anlytical solution assumes ideal rigid bodies.

However, the numerical solution should converge to the analytical solution as the penetration stiffness is increased provided that
a small enough time step is used so that the solution is stable.

This is assessed by performing a convergence test in two different axes:

 1. In time: time step is sistematically decreased for a fixed penetration stiffness.
 2. In penetration stiffness: penetration stiffness is sistematically increased for a fixed time step.

The results for this convergence study are reported in the file `convergence_test`.
