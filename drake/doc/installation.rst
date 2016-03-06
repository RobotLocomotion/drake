
***************************
Installation and Quickstart
***************************


Choose an installation method
=============================

You can choose to download a pre-compiled version of Drake, or to build it from source.  The pre-compiled versions are much easier to use, but the source version is easier to update and will make it easier for you to contribute your fixes/improvements.

.. toctree::
	:maxdepth: 1

	from_binary
	from_source


Optional: Installing MATLAB
===========================

Drake is transitioning from being primarily a MATLAB-based toolbox to a C++ toolbox.  To use the MATLAB interface, we require

* MATLAB R2012a or later

Many of the features also use

* Simulink
* Simulink 3D Animation Toolbox
* Control Systems Toolbox
* Curve Fitting Toolbox
* Optimization Toolbox (as a fallback alternative to commercial solvers)

A number of other toolboxes are used by some Drake algorithms/examples.  You can check your current MATLAB installation using MATLAB's ``ver`` command.

