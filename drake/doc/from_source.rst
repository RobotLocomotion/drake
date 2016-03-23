*****************************************
Source installation (mac, linux, windows)
*****************************************

Optional: Setting up the MATLAB Compiler
========================================

If you'd like to build the MATLAB interface, you will need a `C/C++ compiler that is compatible with your MATLAB version <http://www.mathworks.com/support/compilers/>`_.  Run ``mex -setup`` at the MATLAB command line to set this up.  We typically use gcc on linux, clang on mac, or Visual Studio 2015 on Windows.

Make sure that the MATLAB executable is in your path.  (e.g., typing ``matlab`` at the system command line should start an instance of MATLAB).  For example, on Mac you might consider ``sudo ln -s /Applications/MATLAB_R2014a.app/bin/matlab /usr/bin/matlab``, or you can actually add the MATLAB/bin directory to your system path.  On windows, you should `edit your path environment variable <http://www.java.com/en/download/help/path.xml>`_ to include e.g. ``C:\Program Files\MATLAB\R2014a\bin``.


Getting Drake
=============

Drake is compliant with the `PODs <http://sourceforge.net/p/pods/home/Home/>`_ software development guidelines.  For convenience, we have created pods or wrapper pods for most of the prerequisites, externals, and solvers that you might want to use with Drake.  Some of these will require you to have access to the software licenses.

Note: If you are using Windows, you will want to make sure that ``git`` is set to `handle cross-platform linefeed issues <https://git-scm.com/book/tr/v2/Customizing-Git-Git-Configuration#idp31554304>`_.  These options appear to be enabled by default in the cygwin installation of ``git``, but must be set manually on the native windows version.

Now run::

	git clone https://github.com/RobotLocomotion/drake.git drake-distro


Note: the build process may encounter problems if you have unusual characters like parentheses in the absolute path to the drake-distro directory (see `#394 <https://github.com/RobotLocomotion/drake/issues/394>`_).


If you want to use the private externals
========================================

Drake includes support for some externals which we are not able to redistribute directly 
(SNOPT, SEDUMI, BERTINI, ...).  In order to have drake locally install these for you, you must have access to the corresponding private github repositories, and must set your machine up with SSH keys.  

Follow the instructions here:
https://help.github.com/articles/generating-ssh-keys/

This capability is primarily meant for members of our team and close collaborators with whom we share licenses.


Mandatory platform specific instructions
========================================

Before running build, you will need to follow the instructions for your host system:

.. toctree::
	:maxdepth: 1

	ubuntu
	homebrew
	macports
	windows
	cygwin

(Note: there has been reported success building on Redhat using the Ubuntu installation instructions, and changing ``apt-get`` to ``yum`` inside the ``install_prereqs.sh``, but we haven't tested that ourselves).


Build the collection
====================

::

	cd drake-distro
	make

**NOTE: do not use sudo here**.
Following the pods guidelines, make will automatically perform a local installation.  Just ``make`` is sufficient, and will prevent problems later.

Feel free to use ``make -j`` if your platform supports it.

To include all of the symbols for debugging purposes, execute:

::

    BUILD_TYPE=Debug make

To include all symbols and get details about the actual compiler and linker commands, execute:

::

    BUILD_TYPE=Debug make VERBOSE=true


Test your installation
======================

Start MATLAB, then at the MATLAB prompt do::

	cd drake-distro/drake
	addpath_drake

Then `cd` into the examples directories and try some things out.  Here are a few fun ones to get you started:

* ``runLQR`` in the ``examples/CartPole`` directory
* ``runLQR`` in the ``examples/Quadrotor2D`` directory
* ``RimlessWheelPlant.run()`` in the ``examples/RimlessWheel`` directory
* ``StateMachineControl.run()`` in the ``examples/PlanarMonopodHopper`` directory

For an exhaustive test (which can take more than an hour to run if you have all of the backend solvers enabled), consider running the following from the system terminal (not the MATLAB terminal) command line::

	cd drake-distro/drake
	make test

Note that this is slow -- it starts a fresh instance of MATLAB for every individual test -- but it is the most robust way to test for issues that can potentially crash MATLAB.  Currently, the make test script is setup to send a summary of the success / failures to our continuous integration server after it finishes all of the jobs.  This was our internal setup, and has been left there for now so that we can help you debug your installations.

If you have problems, please check the :doc:`faq`.  If the solution is not there, or if you discover something missing from our installation instructions or lists of prerequisites, then please `file an issue <https://github.com/RobotLocomotion/drake/issues/new>`_ and label it as *installation*.

Stay up to date
===============


To work on the bleeding edge, do::

	cd drake-distro
	git checkout master
	git pull
	make

This is especially useful if you are ready to contribute your work back to the main repository with a `pull request <https://help.github.com/articles/using-pull-requests/>`_.


