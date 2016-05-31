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
$ cd [drake-distro]/drake/examples/Cars
$ ../../../build/bin/drake-visualizer
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

To run the Steering Command Driver, first update your `PYTHONPATH` environment
variable to include Drake's libraries:

```
$ export PYTHONPATH="[path to drake-distro]/build/lib/python2.7/dist-packages:[path to drake-distro]/build/lib/python2.7/site-packages:$PYTHONPATH"
```

Then execute:

```
$ cd [drake-distro]/drake/examples/Cars
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
$ cd [drake-distro]/drake/examples/Cars
$ ../../pod-build/bin/car_sim_lcm models/prius/prius.urdf models/stata_garage_p1.sdf
```

### Simulation Using Drake + LCM + ROS

Drake's car example includes a ROS package called `drake_cars_examples`, which
is located in `[drake distro]/drake/examples/Cars/ros_packages`.
To enable this package, edit your `~/.bashrc` and add the following lines to it
(be sure all other ROS-related settings are commented out):

```
# Setup Drake's ROS workspace
source /opt/ros/indigo/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:[drake-distro]/drake/examples/Cars/ros_packages
```

Save and close your `~/.bashrc` file. Then apply the changes by executing the
following command:

```
$ source ~/.bashrc
```

Ensure the `drake_cars_examples` ROS package is in your ROS workspace. This can
be done by executing the command below. It should change your present
current working directory to be
`[drake distro]/drake/examples/Cars/ros_packages/drake_cars_examples/`.

```
$ roscd drake_cars_examples
```

Start RViz:

```
$ roslaunch drake_cars_examples rviz_prius.launch
```

To run the simulation:

```
$ cd [drake-distro]/drake/examples/Cars
$ ../../pod-build/bin/car_sim_lcm_and_ros models/prius/prius_with_lidar.sdf models/stata_garage_p1.sdf
```

To view the ROS `TF` tree:

```
$ rosrun rqt_tf_tree rqt_tf_tree
```

Another way to view the ROS `TF` tree:

```
$ rosrun tf view_frames
$ evince frames.pdf
```

To drive the car around using a keyboard:

```
$ cd [drake distro]/drake/examples/Cars/ros_packages/
$ git clone git@github.com:liangfok/ackermann-drive-teleop.git ackermann_drive_teleop
$ cd ackermann_drive_teleop
$ git checkout feature/ackermann_drive_stamped
$ rosrun ackermann_drive_teleop ackermann_drive_keyop.py 1.0 0.7
```
The 1.0 and 0.7 parameters in the last command above specifies a max throttle
of 1.0 m/s and a max steering angle of 0.7 radians.

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
$ cd [path to drake-distro]/drake/examples/Cars
$ python steering_command_driver.py --mode=one-time --throttle=[throttle_value] --steering-angle=[steering_value]
```
where the values in square brackets should be replaced with desired values.

For example:

```
$ cd [path to drake-distro]/drake/examples/Cars
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

TODO(jwnimmer-tri) OS X `readlink -f` doesn't work, so the demo script
fails.  Fix and test the demo script on OS X.

Run:
```
$ drake-distro/drake/examples/Cars/simple_car_demo.sh
```

Ensure that the (very small) `pygame` window has focus, then use your
arrow keys and/or joystick to drive around.

Use Ctrl-C in your terminal to stop and close the demo.

Running the trivial multiple car simulator
------------------------------------------

The following notes are for Ubuntu Linux and OS X users.
This is not supported under Windows (though you can probably cobble
together some workarounds by hand if you are motivated).

TODO(jwnimmer-tri) OS X `readlink -f` doesn't work, so the demo script
fails.  Fix and test the demo script on OS X.

Run:
```
$ drake-distro/drake/examples/Cars/run_demo_multi_car.sh [N]
```

This will start the demo with N cars; if N is not supplied, the
default is 100 (and the minimum N is 1).  There are no controls.

Use Ctrl-C in your terminal to stop and close the demo.

### Troubleshooting

You may encounter the following error when executing the command to start RViz:

```
$ roslaunch drake_cars_examples rviz_prius.launch
...
  File "/opt/ros/indigo/share/xacro/xacro.py", line 47, in <module>
    cur_dir = os.getcwd()
OSError: [Errno 2] No such file or directory

```

This is often because the terminal was in a directory that was removed and
replaced with another identically-named directory, possibly due to switching git
branches. To fix this problem, simply change back to your `[drake distro]`
directory and then navigate back to `[drake distro]/examples/Cars`.
