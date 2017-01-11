Demo Instructions
=================

These instructions only support the Bazel build system (not CMake).  For
getting started with Bazel, see http://drake.mit.edu/bazel.html.

Note that the *libraries* in this directory subtree support the CMake build
system; only the demo is Bazel-specific.

Running the demo
----------------

To run the demo, open a terminal and execute the following commands:

```
$ cd drake-distro
$ bazel run /drake/automotive:demo
```

End the demo by closing any window, or Ctrl-C in the terminal.  All of the
launched programs will be closed.

Driving the Prius
-----------------

Ensure that the (very small) `pygame` window has focus, then use your arrow
keys to drive around.  If you have a joystick / steering wheel, you can use
that, too (see `steering_command_driver.py` for details).

Alternatively, you can generate throttle and steering commands using the
command line, for example:

```
$ cd drake-distro
$ bazel-bin/drake/automotive/steering_command_driver --mode=one-time --throttle=1.0 --steering-angle=0.4
```

Running the dynamics
--------------------

Open a new terminal and execute the following:

```
$ cd drake-distro
$ bazel build drake/automotive:demo drake/automotive:car_sim_lcm
$ bazel-bin/external/drake_visualizer/drake-visualizer &
$ bazel-bin/drake/automotive/steering_command_driver &
$ bazel run drake/automotive:car_sim_lcm
```

Then drive the car using the `pygame` window per "Driving the Prius" above.

You can also add a speed bump with a command-line switch:

```
$ bazel run drake/automotive:car_sim_lcm -- --with_speed_bump
```

Enable Chase Cam Mode in the Drake Visualizer
---------------------------------------------

To avoid the car moving out of view within the Drake Visualizer, we recommend
setting Drake Visualizer to chase cam mode.  To do this, select Menu, 'View`,
and 'Camera Control Panel'. Within this control panel, click on 'Select Target',
and then click on the Toyota Prius.  In the control panel, change 'Track Mode'
to be 'Smooth Follow' and increase the elevation to 30 degrees.
