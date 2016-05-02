Car Simulation Instructions
===========================

This README file provides instructions on how to run the car simulations.

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
$ cd [drake-distro]/drake/examples/Cars
$ ../../../build/bin/drake-visualizer &
```

The Drake Visualizer window should appear.

Start the Steering Command Driver
---------------------------------

The Steering Command Driver provides a Graphical User Interface (GUI) for users
to issue driving commands to the car in the simulation. Note that running this
is not strictly necessary since it's possible to issue driving commands directly
from the command line (this will be desribed later in this document).

The Steering Command Driver is based on `pygame`, which can be installed by
executing the following:

```
// On OS X
$ brew install pygame

// On Ubuntu Linux
$ apt-get install python-pygame
```

To run the Steering Command Driver, first update your `PYTHONPATH` environment
variable to include Drake's libraries:

```
$ export PYTHONPATH="[path to drake-distro]/build/lib/python2.7/dist-packages:[path to drake-distro]/build/lib/python2.7/site-packages:$PYTHONPATH"
```

Then execute:

```
$ cd [drake-distro]/drake/examples/Cars
$ python SteeringCommandDriver.py
```

Start the Drake Simulator
-------------------------

There are two version of Drake's cars simulator, one that integrates only
LCM-powered components (e.g., the Drake Visualizer) and a second that that
integrates both LCM-powered components and ROS-powered components (e.g., RViz).

### Simulation Using Drake + LCM

To start the simulation, open a new terminal and execute the following:

```
$ cd [drake-distro]/drake/examples/Cars
$ ../../pod-build/bin/carSimLCM prius/prius.urdf stata_garage_p1.sdf
```

### Simulation Using Drake + LCM + ROS

Drake's car example includes a ROS package located in
`[drake distro]/drake/examples/Cars/ros_packages`.
To enable this package, edit your `~/.bashrc` and add the following lines to it
(be sure all other ROS-related settings are commented out):

```
# Setup Drake's ROS workspace
source /opt/ros/indigo/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:[drake-ditro]/drake/examples/Cars/ros_packages
```

Save and close your `~/.bashrc` file. Then apply the changes by executing the
following command:

```
$ source ~/.bashrc
```

Ensure Drake's `drake_cars_example` ROS package is in your ROS workspace:

```
$ roscd drake_cars_example/
```

To run the simulation:

```
$ cd [drake-distro]/drake/examples/Cars
$ ../../pod-build/bin/carSimLCMandROS prius/prius.sdf stata_garage_p1.sdf
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
$ cd [drake-distro]/drake/examples/Cars
$ ../../pod-build/bin/publishDrivingCommand [throttle_value] [steering_value]
```
where the values in square brackets should be replaced with desired values.

For example:

```
$ cd [path to drake-distro]/drake/examples/Cars
$ ../../pod-build/bin/publishDrivingCommand 1.0 .4
```

Every time that you run the command above, it sends one LCM message.


Adjustments for Windows
-----------------------
- Insert the configuration directory (e.g. `Release/`) after `bin/` in paths to
the executables.
- When running from the Windows Command Prompt you'll need to use backslashes in
place of forward slashes.
- To run a command in the background use `start cmdline` in place of `cmdline &`.
