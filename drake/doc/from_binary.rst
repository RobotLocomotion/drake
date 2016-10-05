********************************
Binary installation (mac, linux)
********************************

Note: This is still new and relatively untested.  Please report any issues you have.

Download the appropriate binary release for your platform
https://github.com/RobotLocomotion/drake/releases
Simply extract the archive file into a folder of your choice (mine is called ``drake-distro``).

Running MATLAB examples
=======================

To run the MATLAB examples, change directories (in MATLAB) into the ``drake-distro/drake`` folder and at the MATLAB prompt do::

	addpath_drake


Then ``cd`` into the examples directories and try some things out.  Here are a few fun ones to get you started:

* ``runLQR`` in the ``examples/CartPole`` directory
* ``runLQR`` in the ``examples/Quadrotor2D`` directory
* ``RimlessWheelPlant.run()`` in the ``examples/RimlessWheel`` directory
* ``StateMachineControl.run()`` in the ``examples/PlanarMonopodHopper`` directory

Please note that you will have to run `addpath_drake` each time you start MATLAB, or `add it to your startup.m <http://www.mathworks.com/help/matlab/ref/startup.html>`_.

Linux Specific
--------------

The version of the standard C++ libraries that are shipped with the Linux distribution of MATLAB is severely outdated and can cause problems when running mex files that are built against a newer version of the standard.  The typical error message in this case reports `Invalid MEX-Files`

To work around this issue, the symbolic link for the standard C++ library provided by MATLAB must be redirected to point to a more up-to-date version.

First, make sure that a suitable version of the standard library is installed::

	sudo apt-get install g++-4.4

Now, the symbolic link in MATLAB must be updated to point to the version that was just installed in `/usr/lib`.  An example for MATLAB R2014a is shown below::

	cd /usr/local/MATLAB/R2014a/sys/os/glnxa64
	sudo rm libstdc++.so.6
	sudo ln -s /usr/lib/gcc/x86_64-linux-gnu/4.4/libstdc++.so libstdc++.so.6
