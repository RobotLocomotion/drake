How to run the car simulation
=============================
 
Additional prerequisite
-----------------------

Install `pygame` (e.g. with `brew install pygame`, or `apt-get install python-pygame`)

Note that you can still run the car simulation without `pygame`, but it won't be as fun.
To run the simulation without `pygame`, use `publish_driving_command` as described below.

Setup your Python path:

```
$ export PYTHONPATH="[path to drake-distro]/build/lib/python2.7/dist-packages:[path to drake-distro]/build/lib/python2.7/site-packages:$PYTHONPATH"
```

Running the prius simulator
---------------------------
The following notes are for Ubuntu Linux and OS X users. Windows users need to adjust the instructions slightly. See the notes at the end of this section.

```
$ cd [path to drake-distro]/drake/examples/Cars
$ ../../../build/bin/drake-visualizer &
$ python SteeringCommandDriver.py &
$ ../../pod-build/bin/car_sim_lcm models/prius/prius.urdf models/stata_garage_p1.sdf
```

To avoid the car moving out of view, we recommend setting Drake Visualizer to chase cam mode.  Choose from the Menu, select 'View`, then 'Camera Control Panel'. Then click 'Select Target' and click on the Toyota Prius.  Change 'Track Mode' to be 'Smooth Follow' and increase the elevation to 30 degrees.

Ensure that the (very small) `pygame` window has focus, then use your arrow
keys to drive around. If you have a joystick / steering wheel.. you can use
that, too (see SteeringCommandDriver.py for details).

If for some reason the python game interface is not to your liking (e.g. you
don’t have python on your system, etc), then you can generate simple throttle
and steering commands using the command line interface

```
$ cd [path to drake-distro]/drake/examples/Cars
$ ../../pod-build/bin/publish_driving_command [throttle_value] [steering_value]
```
where the values in brackets should be replaced with desired values.  For example:

```
$ cd [path to drake-distro]/drake/examples/Cars
$ ../../pod-build/bin/publish_driving_command 1.0 .4
```
Every time that you run the command above, it sends one LCM message.

*Adjustments for Windows*
- Insert the configuration directory (e.g. `Release/`) after `bin/` in paths to
the executables.
- When running from the Windows Command Prompt you'll need to use backslashes in
place of forward slashes.
- To run a command in the background use `start cmdline` in place of `cmdline &`.

Running the simple car simulator
--------------------------------

Run `./simple_car_demo.sh`.

This is not supported under windows (though you can probably cobble
together some workarounds by hand if you are motivated).
