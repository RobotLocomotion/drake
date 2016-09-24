Car Simulation Instructions
===========================

This README file provides instructions on how to run Drake's car simulations.

The instructions are written for Ubuntu Linux and OS X users. Windows users will
need to adjust the instructions slightly. See the notes at the end of this
section.
 
Start the Drake Visualizer
--------------------------

The Drake Visualizer displays the current state of the simulation. It is a
separate process that communicates with the Drake simulation process via the
[Lightweight Communications and Marshalling (LCM)](https://lcm-proj.github.io/)
middleware.

To run the Drake Visualizer, open a terminal and execute the following commands:

```
$ cd drake-distro/drake/automotive
$ ../../build/install/bin/drake-visualizer
```

The Drake Visualizer window should appear.

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

Start the Drake Simulator
-------------------------

There is currently one version of Drake's cars simulator. It integrates only
LCM-based components (e.g., the Drake Visualizer). In the future, a second version
will be added that integrates both LCM-based components and ROS-based components
(e.g., RViz).

### Simulation Using Drake + LCM

To start the simulation, open a new terminal and execute the following:

```
$ cd drake-distro/drake/automotive
$ ../../build/drake/bin/car_sim_lcm models/prius/prius.urdf models/stata_garage_p1.sdf
```

### Simulation Using Drake + LCM + ROS

See: https://github.com/liangfok/drake/tree/feature/multi_car_sim_2/ros/drake_cars_examples

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


Adjustments for Windows
-----------------------
- Insert the configuration directory (e.g. `Release/`) after `bin/` in paths to
the executables.
- When running from the Windows Command Prompt you'll need to use backslashes in
place of forward slashes.
- To run a command in the background use `start cmdline` in place of `cmdline &`.

Running the simple car simulator
--------------------------------

The following notes are for Ubuntu Linux and OS X users.
This is not supported under Windows (though you can probably cobble
together some workarounds by hand if you are motivated).

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
