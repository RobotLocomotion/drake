Demo Instructions
=================

These instructions only support the Bazel build system (not CMake).  For
getting started with Bazel, see http://drake.mit.edu/bazel.html.

Running the demos
-----------------

A variety of demos are available.  In general, to run a demo, open a
terminal and execute commands like the ones shown below. End the demo by closing
any window, or Ctrl-C in the terminal.  All of the launched programs will be
closed.

```
$ cd drake
$ bazel run automotive:DEMO_NAME_HERE
```

The following demos are available:

 * One `SimpleCar` under user control and one `TrajectoryCar` driving around in
   a figure eight on an open plane:

   ```
   bazel run automotive:demo -- --num_simple_car=1 \
       --driving_command_gui_names=0 --num_trajectory_car=1
   ```

   This will show one _ado_ car driving in a fixed trajectory, and one _ego_
   car which can be driven anywhere on the infinite plane.  (See "Driving
   the Prius" below to make it go.)

 * A 3-lane dragway with four `TrajectoryCar` vehicles traveling down each lane
   at different speeds plus one `SimpleCar` and one `MaliputRailcar`:

   ```
   bazel run automotive:demo -- \
       --num_dragway_lanes=3 \
       --num_trajectory_car=12 \
       --num_maliput_railcar=1
   ```

Driving the Prius
-----------------

Ensure that the (very small) `pygame` window has focus, then use your arrow
keys to drive around.  If you have a joystick / steering wheel, you can use
that, too (see `steering_command_driver.py` for details).

Alternatively, you can generate throttle and steering commands using the
command line, for example:

```
$ cd drake
$ bazel-bin/automotive/steering_command_driver --mode=one-time --throttle=1.0 --steering-angle=0.4
```

Running the dynamics
--------------------

The following instructions describe how to run `car_sim_lcm` — a
dynamics-based simulation of a sedan that approximates a Toyota Prius. The model
is simplistic; it does not model engine, transmission, or suspension
dynamics, but it is still useful in terms of illustrating Drake's dynamics-based
simulation capability. The model consists of four wheels where the front wheels
are steered using
[Ackermann steering](https://en.wikipedia.org/wiki/Ackermann_steering_geometry)
and the rear wheels are fixed to an axle that spans the width of the
vehicle (they rotate passively). In addition to being steerable, the front
wheels also have velocity-controlled actuators that enable the vehicle to move
forward and backward. PID controllers are used to control both the steering
angle and front wheel rotational velocities. A simple sliding friction model is
used for the contacts between the wheels and the ground. The input reference
values for the PID controllers are settable via LCM using
`lcmt_driving_command_t` messages published on channel "DRIVING_COMMAND". The
current state of the simulation can be visualized in Drake Visualizer.

To run `car_sim_lcm`, open a new terminal and execute the following commands:

```
$ cd drake
$ bazel build automotive:demo automotive:car_sim_lcm
$ bazel-bin/tools/drake_visualizer &
$ bazel-bin/automotive/steering_command_driver &
$ bazel run automotive:car_sim_lcm
```

Then drive the car using the `pygame` window per "Driving the Prius" above.

You can also add a speed bump with a command-line switch:

```
$ bazel run automotive:car_sim_lcm -- --with_speed_bump
```

Enable Chase Cam Mode in the Drake Visualizer
---------------------------------------------

To avoid the car moving out of view within the Drake Visualizer, we recommend
setting Drake Visualizer to chase cam mode.  To do this, select Menu, 'View`,
and 'Camera Control Panel'. Within this control panel, click on 'Select Target',
and then click on the Toyota Prius.  In the control panel, change 'Track Mode'
to be 'Smooth Follow' and increase the elevation to 30 degrees.
