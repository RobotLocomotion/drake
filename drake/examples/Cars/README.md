How to run the car simulation
=============================
 
Additional prerequisite
-----------------------

Install pygame (e.g. with `brew install pygame`, or `apt-get install python-pygame`)

Set up your python path, e.g. with

```
export PYTHONPATH="/path/to/drake-distro/build/lib/python2.7/dist-packages:/path/to/drake-distro/build/lib/python2.7/site-packages:$PYTHONPATH"
```
e.g. for me it is

```
export PYTHONPATH="/Users/russt/drake-distro/build/lib/python2.7/dist-packages:/Users/russt/drake-distro/build/lib/python2.7/site-packages:$PYTHONPATH"
```

You can run the car simulation without pygame; you won't have much fun driving
it though. In that case you can use publishDrivingCommand (see below) to control it.


Running the simulator
---------------------
To adjust these instructions for Windows, see notes at the end of this section.

```
cd drake-distro/drake/examples/Cars
../../../build/bin/drake-visualizer &
python SteeringCommandDriver.py &
../../pod-build/bin/carSimLCM prius/prius.urdf stata_garage_p1.sdf
```

I recommend setting the viewer to chase cam mode.  Choose from the Menu 'View->Camera Control Panel'.
Then click 'Select Target' and click on the prius.  Change track mode to 'Smooth Follow' and increase your elevation.

Make sure that the (very small) pygame window has focus, then use your arrow
keys to drive around. If you have a joystick / steering wheel.. you can use
that, too (see SteeringCommandDriver.py for details).

If for some reason the python game interface is not to your liking (e.g. you
don’t have python on your system, etc), then you can generate simple throttle
and steering commands using the command line interface

```
../../pod-build/bin/publishDrivingCommand [throttle_value] [steering_value]
```
where the values in bracket should be replaced with desired values.  e.g.

```
../../pod-build/bin/publishDrivingCommand 1.0 .4
```
Every time that you run this command, it sends one LCM message.

*Adjustments for Windows*
- Insert the configuration directory (e.g. `Release/`) after `bin/` in paths 
below to find executables. 
- When running from the Windows Command Prompt you'll need to use backslashes in
place of forward slashes.
- To run a command in the background use `start cmdline` in place of `cmdline &`.
