How to run the car simulation
=============================
 
Additional prerequisite
-----------------------

Install `pygame` (e.g. with `brew install pygame`, or `apt-get install python-pygame`)

Note that you can still run the car simulation without `pygame`, but it won't be as fun.
To run the simulation without `pygame`, use `publishDrivingCommand` as described below.

Add environment variable `DRAKE_ROOT` pointing to the location of your clone of the Drake repository.

```
$ cd [path to drake-distro]
$ export DRAKE_ROOT=`pwd`
```

Setup your Python path:

```
$ export PYTHONPATH="$DRAKE_ROOT/build/lib/python2.7/dist-packages:$DRAKE_ROOT/build/lib/python2.7/site-packages:$PYTHONPATH"
```

Running the simulator
---------------------
The following notes are for Ubuntu Linux and OS X users. Windows users need to adjust the instructions slightly. See the notes at the end of this section.

```
$ cd $DRAKE_ROOT/drake/examples/Cars
$ ../../../build/bin/drake-visualizer &
$ python SteeringCommandDriver.py &
$ ../../pod-build/bin/carSimLCM prius/prius.urdf stata_garage_p1.sdf
```

To avoid the car moving out of view, we recommend setting Drake Visualizer to chase cam mode.  Choose from the Menu, select 'View`, then 'Camera Control Panel'. Then click 'Select Target' and click on the Toyota Prius.  Change 'Track Mode' to be 'Smooth Follow' and increase the elevation to 30 degrees.

Ensure that the (very small) `pygame` window has focus, then use your arrow
keys to drive around. If you have a joystick / steering wheel.. you can use
that, too (see SteeringCommandDriver.py for details).

If for some reason the python game interface is not to your liking (e.g. you
don’t have python on your system, etc), then you can generate simple throttle
and steering commands using the command line interface

```
$ ../../pod-build/bin/publishDrivingCommand [throttle_value] [steering_value]
```
where the values in brackets should be replaced with desired values.  For example:

```
$ ../../pod-build/bin/publishDrivingCommand 1.0 .4
```
Every time that you run the command above, it sends one LCM message.

*Adjustments for Windows*
- Insert the configuration directory (e.g. `Release/`) after `bin/` in paths to 
the executables.
- When running from the Windows Command Prompt you'll need to use backslashes in
place of forward slashes.
- To run a command in the background use `start cmdline` in place of `cmdline &`.
