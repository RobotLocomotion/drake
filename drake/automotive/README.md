Demo Instructions
=================

These instructions only support the Bazel build system (not CMake).  For
getting started with Bazel, see http://drake.mit.edu/bazel.html.

Note that the *libraries* in this directory subtree support the CMake build
system; only the demo is Bazel-specific.

Prepare the drake-visualizer
----------------------------

The ``drake-visualizer`` is only available via a CMake build.  We recommend
that you run a CMake build per the Drake instructions, using the same source
tree as this demo.  In that case, the ``drake-visualizer`` will automatically
be discovered and launched by this demo.

Otherwise, you will see a message like:

```
soft_failure.bzl: @drake_visualizer//:drake-visualizer does not work because
  drake-distro/build/install/bin/drake-visualizer was missing
```

In this case, the automotive demo must be run with the ``--no-visualizer``
switch, and you will have to manually launch some other build of the
``drake-visualizer``.

Running the demos
-----------------

A variety of demos are available.  In general, to run a demo, open a
terminal and execute commands like this:

```
$ cd drake-distro
$ bazel run drake/automotive:DEMO_NAME_HERE
```

End the demo by closing any window, or Ctrl-C in the terminal.  All of the
launched programs will be closed.

 * Basic cars driving around on an open plane:
   `bazel run drake/automotive:demo`

   This will show one _ado_ car driving in a fixed trajectory, and one _ego_
   car which can be driven anywhere on the infinite plane.  (See "Driving
   the Prius" below to make it go.)

 * Original "Maliput Village" demo, showing behavior at intersections:
   `bazel run drake/automotive:village-demo`

   This will show 15 _ado_ cars driving on a fixed, but self-intersecting,
   path through the "Maliput Village" road network.  One can add a driveable
   _ego_ car as well, by adding an extra argument:
   `bazel run drake/automotive:village-demo -- -use_ego_car=true`
   The _ego_ car is constrained to drive on the same path as the _ado_ cars,
   though its velocity and heading can be controlled by the user.

 * "Maliput Village" demo showing merging behavior:
   `bazel run drake/automotive:merging-demo`

   Same general idea as the `village-demo`, but with a different path for
   cars which includes a merge.

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
