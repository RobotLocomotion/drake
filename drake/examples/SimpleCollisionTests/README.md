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

