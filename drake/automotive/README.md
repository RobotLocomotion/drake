Car Simulation Instructions
===========================

This README file provides instructions on how to run Drake's car simulations.
 
Start Drake's Visualizer
------------------------

Drake's visualizer displays the current state of the simulation. It is a
separate process that communicates with Drake's simulation via the
[Lightweight Communications and Marshalling (LCM)](https://lcm-proj.github.io/)
middleware.

To run Drake's visualizer, open a terminal and execute the following commands:

```
$ drake-distro/build/install/bin/drake-visualizer
```

Drake's visualizer should appear.

Start the Steering Command Driver
---------------------------------

The Steering Command Driver provides a Graphical User Interface (GUI) for users
to issue driving commands to the car in the simulation. Note that running this
is not strictly necessary since it's possible to issue driving commands directly
from the command line. To run the simulation without `pygame`, use `--mode=one-time`
as described below.

The Steering Command Driver is based on `pygame`, which can be installed by
executing the following:

```
// On OS X
$ brew install pygame

// On Ubuntu Linux
$ apt-get install python-pygame
```

Then execute:

```
$ cd drake-distro/drake/automotive
$ python steering_command_driver.py
```

Start Drake's Simulator
-----------------------

There is currently one physics-based car simulation called `car_sim_lcm`. This
simulation can be run in two modes: "normal" and "speed bump". The "normal" mode
consists of a Toyota Prius on a flat terrain. The "speed bump" mode is the same
as the "normal" mode except there is a speed bump placed in front of the
vehicle. See the instructions below on how to run each.

Running car_sim_lcm in Normal Mode
==================================

Open a new terminal and execute the following:

```
$ drake-distro/build/drake/automotive/car_sim_lcm
```

Running car_sim_lcm in Speed Bump Mode
======================================

Open a new terminal. Then execute the following command to generate the
`speed_bump.obj` file using `maliput`:

```
$ cd drake-distro/build
$ ./drake/drake/automotive/maliput/yaml_to_obj --yaml_file ../drake/automotive/models/speed_bump/speed_bump.yaml --obj_file ../drake/automotive/models/speed_bump/speed_bump.obj
```

Next execute the following to start the simulation:

```
$ drake-distro/build/drake/automotive/car_sim_lcm --with_speed_bump
```

Additional Simulation Notes
---------------------------

### Enable Chase Cam Mode in the Drake Visualizer

To avoid the car moving out of view within the Drake Visualizer, we recommend
setting Drake Visualizer to chase cam mode.  To do this, select Menu, 'View`,
and 'Camera Control Panel'. Within this control panel, click on 'Select Target',
and then click on the Toyota Prius.  In the control panel, change 'Track Mode'
to be 'Smooth Follow' and increase the elevation to 30 degrees.

### Driving the Prius Around in the Simulation

If you're running the Steering Command Driver, ensure that the (very small)
`pygame` window has focus, then use your arrow keys to drive around. If you have
a joystick / steering wheel.. you can use that, too (see
`SteeringCommandDriver.py` for details).

If you are unable to run the Steering Command Driver, you can generate simple
throttle and steering commands using the command line:

```
$ cd drake-distro/drake/automotive
$ python steering_command_driver.py --mode=one-time --throttle=[throttle_value] --steering-angle=[steering_value]
```
where the values in square brackets should be replaced with desired values.

For example:

```
$ cd drake-distro/drake/automotive
$ python steering_command_driver.py --mode=one-time --throttle=1.0 --steering-angle=0.4
```

Every time that you run the command above, it sends one LCM message.

Running the Simple Car Simulator
--------------------------------

Run:
```
$ drake-distro/drake/automotive/automotive_demo.py
```

Ensure that the (very small) `pygame` window has focus, then use your
arrow keys and/or joystick to drive around.

TODO(jwnimmer-tri) The trajectory car(s) are not visualized yet.  They only
show up in the bot-spy status.

Use Ctrl-C in your terminal to stop and close the demo.

You can also pass --num_trajectory_car=100 to the script to add more cars.
