.. _binary-installation:

***********************************
Binary installation (macOS, Ubuntu)
***********************************

Important Note (October, 2016)
==============================

Drake is currently undergoing a major renovation, with all of the core
libraries moving into C++.  The examples will move and the existing APIs will
change.

There are `experimental binary packages <https://github.com/RobotLocomotion/drake/issues/1766#issuecomment-318955338>`_ of Drake available at:

- https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-xenial.tar.gz
- https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-mac.tar.gz
- :samp:`https://drake-packages.csail.mit.edu/drake/nightly/drake-{yyyymmdd}-xenial.tar.gz`.
    - Example: https://drake-packages.csail.mit.edu/drake/nightly/drake-20171015-xenial.tar.gz

Example usages of these binaries are shown in this `example CMake project <https://github.com/RobotLocomotion/drake-shambhala/tree/master/drake_cmake_installed>`_.
For the compilers used to produce these releases, see :ref:`binary-packages`.
If you are unsure of which approach to use, we suggest you :ref:`build from source <build_from_source>`
instead.

Using older (2015) releases
===========================

We cannot offer support for older releases, but we still host the binaries for
reference.

Download the appropriate binary release for your platform
https://github.com/RobotLocomotion/drake/releases
Simply extract the archive file into a folder of your choice (mine is called ``drake-distro``).


Running MATLAB examples
-----------------------

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
