.. _faq:

**************************
Frequently Asked Questions
**************************

.. contents:: `Table of contents`
   :depth: 3
   :local:

.. _faq_vmware:

Why doesn't Drake Visualizer work in VMWare Fusion or Workstation?
==================================================================

Symptom: The simulation runs and the visualization window appears, but no
objects are actually drawn. This appeared to be due to display drivers and/or
non support of hardware-accelerated rendering. To address this, go to
``Virtual Machine Settings``, and check the ``Accelerate 3D Graphics`` box under
Display settings; now the simulations draw properly.

.. _faq_drake_visualizer_no_module_named_vtk_common_core_python:

Why does Drake Visualizer crash with a "No module named vtkCommonCorePython" Error?
===================================================================================

Symptom: When you start ``drake-visualizer``, it crashes with the following
error::

    File "/usr/lib/python2.7/dist-packages/vtk/__init__.py", line 39, in <module>
      from vtkCommonCore import *
    File "/usr/lib/python2.7/dist-packages/vtk/vtkCommonCore.py", line 1, in <module>
      from vtkCommonCorePython import *
    ImportError: No module named vtkCommonCorePython

Solution: This is a `known problem <https://github.com/RobotLocomotion/drake/issues/4738>`_
when you have ``python-vtk6`` installed. The workaround depends on whether
you're building Drake within a ROS Catkin workspace. Please jump to the
appropriate subsection below.

.. _faq_drake_visualizer_no_module_named_vtk_common_core_python_non_ros:

Non-ROS Users
-------------

To workaround the problem, configure Director's build system to build VTK5 from
source (``drake-visualizer`` is built on Director, which is built on VTK5)::

    cd drake-distro/build/externals/director
    cmake . -DUSE_SYSTEM_VTK=OFF
    cd drake-distro/build
    make (or ninja)

Next, modify two environment variables before starting ``drake-visualizer``
(note that this has only been tested when there are no spaces in the path to
the present working directory)::

    cd drake-distro
    export LD_LIBRARY_PATH=`pwd`/build/install/lib/vtk-5.10:$LD_LIBRARY_PATH
    export PYTHONPATH=`pwd`/build/externals/director/src/vtk-build/Wrapping/Python:`pwd`/build/externals/director/src/vtk-build/bin:$PYTHONPATH

You should now be able to start ``drake-visualizer``.


.. _faq_drake_visualizer_no_module_named_vtk_common_core_python_ros_indigo:

ROS Indigo Users
----------------

To workaround the problem, configure Director's build system to build VTK5 from
source (``drake-visualizer`` is built on Director, which is built on VTK5)::

    cd ~/dev/drake_catkin_workspace/build/drake/externals/director
    cmake . -DUSE_SYSTEM_VTK=OFF
    cd ~/dev/drake_catkin_workspace
    catkin build

Next, modify two environment variables before starting
``drake-visualizer``::

    export LD_LIBRARY_PATH=$HOME/dev/drake_catkin_workspace/install/lib/vtk-5.10:$LD_LIBRARY_PATH
    export PYTHONPATH=$HOME/dev/drake_catkin_workspace/build/drake/externals/director/src/vtk-build/Wrapping/Python:$HOME/dev/drake_catkin_workspace/build/drake/externals/director/src/vtk-build/bin:$PYTHONPATH

You should now be able to start ``drake-visualizer``.


.. _faq_drake_visualizer_no_module_named_vtk_common_core_python_ros_kinetic:

ROS Kinetic Users
-----------------

Since ROS Kinetic requires VTK6, the
:ref:`setup instructions <build_from_source_using_ros_kinetic>` already contain
the workaround to this error. See in particular
:ref:`step 5 <drake_ros_kinetic_build_workspace>` and
:ref:`step 6 <drake_ros_kinetic_environment_variables>`.

.. _faq_java_classes:

Why don't my Java classes to work in Drake (e.g., after running make)?
======================================================================

This could be an issue with the version of the Java compiler (javac)
installed on your system. MATLAB currently cannot run classes that were
compiled with the Java 1.7 (or free alternative compilers that are version 1.7
compliant). You should retarget your compiling for version 1.6. This can be done
by passing javac the ``--source=1.6 --target=1.6`` flags.

.. _faq_java_matlab_2013:

Error building Drake w/ MATLAB R2013a after switching to Mavericks
==================================================================

Symptoms: When running ``make`` in Drake, you get the following error messages::

	xcodebuild: error: SDK "macosx10.7" cannot be located.
	xcrun: error: unable to find utility "clang", not a developer tool or in PATH
	-- compiler1 version string:
	-- compiler2 version string: 4.2.1
	CMake Error at cmake/mex.cmake:203 (message):
		Your cmake C compiler is: /usr/bin/cc but your mex options use: xcrun -sdk
		macosx10.7 clang .  Consider rerunning 'mex -setup' in Matlab.

Cause: Matlab's ``mexopts.sh`` in the bin folder of your MATLAB installation
statically refers to the 10.7 sdk, which was removed in Mavericks .

Fix: Replace all occurrences of ``10.7`` in mexopts.sh by ``10.9``. After this,
run ``mex -setup`` in Matlab and select the option that mentions mexopts.sh
(option 1 in my case).

http://stackoverflow.com/questions/20294160/matlab-error-regarding-compile-mex-command

.. _faq_compile_mex:

Can't compile mex after upgrading XCode to 5.1 on Mac.
======================================================

   http://www.mathworks.com/matlabcentral/answers/121305-mex-cpp-under-matlab-2013b-and-xcode-5-0
   http://stackoverflow.com/questions/22367516/matlab-mex-compile-error

The error message looks like::

	/Applications/MATLAB_R2012a.app/extern/include/tmwtypes.h:819:9: error: unknown type name 'char16_t'
	typedef char16_t CHAR16_T;

MATLAB's types are not compatible with the newest version of clang.  Hopefully
they will get in sync soon, but for now I've decided the best fix is to edit the
``twmtypes.h`` file::

	/*typedef char16_t CHAR16_T;*/
	typedef UINT16_T CHAR16_T;


.. _faq_simulink_not_found:

Drake tells me I don't have Simulink 3D Animation Toolbox, but I'm sure that I do!
==================================================================================

You might have to actually tell MATLAB to install the tool, running ``vrinstall`` in MATLAB.

.. _faq_undefined_symbol-sincos_stret:

Undefined symbol "___sincos_stret" on Mac.
==========================================

This is an optimization in the XCode 5.  Update your ``mexopts.sh`` to make sure your ``MACOSX_DEPLOYMENT_TARGET`` is set to 10.9.  (It's best to just search and replace 10.8 for 10.9)


.. _faq_ubuntu_nvidia:

Problems Installing Drake on Ubuntu After Installing NVidia Drivers
===================================================================

I'm having trouble trying to install drake on Ubuntu after installing NVidia drivers. I get the error::

	make[5]: *** No rule to make target `/usr/lib/x86_64-linux-gnu/libGL.so', needed by `lib/libbot2-frames-renderers.so.1'.  Stop.

You may need to follow these steps:
http://techtidings.blogspot.com/2012/01/problem-with-libglso-on-64-bit-ubuntu.html

.. _faq_jpeglib:

Can't find ``jpeglib.h`` when compiling ``bot2-vis`` on Mac
===========================================================

Make sure you've installed the xcode command line tools with ``xcode-select --install``, then ``make clean`` and ``make`` again.

.. _faq_LCM_singleton_fail:

MATLAB SocketException error when calling LCM
=============================================

In MATLAB on OSX Yosemite, you may see the following error when calling LCM::

	"LC singleton fail: java.net.SocketException: Can't assign requested address"

Apply the resolution described here: https://github.com/RobotLocomotion/drake/issues/558

.. _faq_mex_cc_not_found:

``Could not find MEX_CC using mex -v`` on Mac after updating XCode.
===================================================================

Open Matlab. Run::

	edit ([matlabroot '/bin/maci64/mexopts/clang_maci64.xml'])

Search for ``MacOSX10.10``. Toward the bottom, you will find four hits.

Two of these hits (one on line 121, one on line 133 of the unmodified file) look like::

	<dirExists name="$$/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.10.sdk" />

In both locations, copy this line, paste it on the next line and change the second one to "10.11", like this::

	<dirExists name="$$/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.10.sdk" />
	<dirExists name="$$/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.11.sdk" />


The other two hits (one on line 123, one on line 135 of the unmodified file) look like::

	<cmdReturns name="find $$ -name MacOSX10.10.sdk" />

Repeat the copy/paste/modify 10.10 to 10.11 process for these lines.

Repeat this whole process for ``clang++_maci64.xml``, ``gfortran.xml``, and ``intel_fortran.xml``.

(note: this is a slightly more thorough version of the resolution described here: http://www.mathworks.com/matlabcentral/answers/243868-mex-can-t-find-compiler-after-xcode-7-update-r2015b ).
