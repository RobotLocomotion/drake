Demo Instructions
=================

These instructions only support the Bazel build system (not CMake).  For
getting started with Bazel, see https://drake.mit.edu/bazel.html.

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

Enable Chase Cam Mode in the Drake Visualizer
---------------------------------------------

To avoid the car moving out of view within the Drake Visualizer, we recommend
setting Drake Visualizer to chase cam mode.  To do this, select Menu, 'View`,
and 'Camera Control Panel'. Within this control panel, click on 'Select Target',
and then click on the Toyota Prius.  In the control panel, change 'Track Mode'
to be 'Smooth Follow' and increase the elevation to 30 degrees.

Code Review
===========

The entire //automotive/... directory tree is exempt from platform review.

For PRs that affect only this directory tree, two feature reviews are
sufficient; platform review is not required.
