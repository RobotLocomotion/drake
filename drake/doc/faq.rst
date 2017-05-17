.. _faq:

**************************
Frequently Asked Questions
**************************

.. contents:: `Table of contents`
   :depth: 3
   :local:

.. _faq_osx_build_failure_missing_dependency_declarations:

Why Does Build Fail with "this rule is missing dependency declarations" on macOS?
=================================================================================

Symptom: After upgrading Xcode on macOS, you encounter an error similar to the
following::

    $ bazel build ...
    ...
    ERROR: /private/var/tmp/_bazel_liang/6afb2531e78184cc48f3db789230c79d/
    external/libbot/BUILD.bazel:59:1: undeclared inclusion(s) in rule
    '@libbot//:ldpc':
    this rule is missing dependency declarations for the following files
    included by 'external/libbot/bot2-lcm-utils/src/tunnel/ldpc/getopt.cpp':
      '/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/
      Developer/SDKs/MacOSX10.12.sdk/usr/include/ctype.h'

Solution: Install Xcode's command-line tools and reset the tools' path to be
the default. To install Xcode's command-line tools::

    $ xcode-select --install

Once installed, you should have ``bin/``, ``include/``, ``lib/``, and
``libexec/`` directories within ``/Library/Developer/CommandLineTools/usr/``.

Check the Xcode command line tools' path::

    $ xcode-select -p

Drake's Bazel-based build system is currently
`hard-coded <https://github.com/RobotLocomotion/drake/blob/c8b974baee3144acecb063607e90287ca009734c/tools/CROSSTOOL#L362-L366>`_
to assume the Xcode command line tools are in the default location of
``/Applications/Xcode.app/Contents/Developer``. If the path is not the
default, reset it to be the default by executing the following command::

    $ sudo xcode-select --reset

.. _faq_missing_or_stray_characters_in_generate_urdf_test:

Why Does Build Fail With Missing or Stray Character Error in generate_urdf_test.cc?
===================================================================================

Symptom: When building Drake using ``cmake`` and ``make`` or ``ninja``, the
following error occurs::

    drake-distro/drake/automotive/maliput/utility/test/generate_urdf_test.cc:88:3: error: missing terminating " character
       EXPECT_EQ(R"R(<?xml version="1.0" ?>
       ^
    drake-distro/drake/automotive/maliput/utility/test/generate_urdf_test.cc:89:1: error: stray ‘\’ in program
     <robot name="dut">
     ^
    drake-distro/drake/automotive/maliput/utility/test/generate_urdf_test.cc:89:1: error: missing terminating " character
    drake-distro/drake/automotive/maliput/utility/test/generate_urdf_test.cc:90:3: error: stray ‘\’ in program
       <link name="world"/>

Solution: One known cause of this problem is employing ``ccache`` without the
environment variable ``CCACHE_CPP2=yes``. Try adding the following line to your
``~/.bashrc``::

    export CCACHE_CPP2=yes

Alternatively, update to
`ccache 3.3 <https://ccache.samba.org/releasenotes.html#_ccache_3_3>`_ or newer
since it by default sets ``CCACHE_CPP2=yes``.

.. _faq_cmake_vtk_version_crash:

Why Does CMake Fail With An Error Indicating No Compatible VTK Found?
=====================================================================

Symptom: When running ``cmake ..`` in ``drake-distro/build``, the following
error is reported::

    CMake Error at cmake/options.cmake:172 (find_package):
      Could not find a configuration file for package "VTK" that is compatible
      with requested version "5.10".

      The following configuration files were considered but not accepted:

        /usr/lib/cmake/vtk-6.2/VTKConfig.cmake, version: 6.2.0

    Call Stack (most recent call first):
      cmake/options.cmake:249 (drake_system_dependency)
      CMakeLists.txt:17 (drake_setup_options)


    -- Configuring incomplete, errors occurred!

Solution: This error is typically caused by your system having a newer version
of VTK installed than the version required by Drake. As indicated by the error
message, Drake needs VTK 5.10 whereas the system has VTK 6.2. To get around this
error, add ``-DUSE_SYSTEM_VTK=OFF -DWITH_VTK=ON`` to the ``cmake`` command::

    cd drake-distro/build
    cmake .. -DUSE_SYSTEM_VTK=OFF -DWITH_VTK=ON

Alternatively, if you have a compatible version of VTK5 installed, you can tell
CMake to use it by specifying ``VTK_DIR`` as follows::

    cd drake-distro/build
    cmake .. -DVTK_DIR=path/to/vtk5

.. _faq_vmware:

Why doesn't Drake Visualizer work in VMWare Fusion or Workstation?
==================================================================

Symptom: The simulation runs and the visualization window appears, but no
objects are actually drawn. This appeared to be due to display drivers and/or
non support of hardware-accelerated rendering. To address this, go to
``Virtual Machine Settings``, and check the ``Accelerate 3D Graphics`` box under
Display settings; now the simulations draw properly.

.. _faq_drake_visualizer_segfault:

Why Does drake-visualizer Segfault Upon Start?
==============================================

Symptom: VTK6 is installed but Drake's CMake-based super-build is configured to
download and build against VTK5 using the technique described in
:ref:`here <faq_cmake_vtk_version_crash>`. When starting ``drake-visualizer``,
it immediately segfaults::

    $ cd drake-distro
    $ ./build/install/bin/drake-visualizer
    Segmentation fault (core dumped)

Solution: The problem is ``drake-visualizer`` is correctly being built against
VTK5, but is incorrectly run against VTK6. To fix this problem, modify the
``LD_LIBRARY_PATH`` and ``PYTHONPATH`` environment variables to ensure VTK5 is
prioritized over VTK6 as described
:ref:`here <faq_drake_visualizer_no_module_named_vtk_common_core_python_non_ros>`.
For more information, see `this comment <https://github.com/RobotLocomotion/drake/issues/5280#issuecomment-282036045>`_.

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

To workaround the problem, configure Drake's build system to build VTK5 from
source (``drake-visualizer`` is built on Director, which is built on VTK5)::

    cd drake-distro/build
    cmake . -DUSE_SYSTEM_VTK=OFF -DWITH_VTK=ON
    cd drake-distro/build
    make (or ninja)

Next, modify two environment variables before starting ``drake-visualizer``
(note that this has only been tested when there are no spaces in the path to
the present working directory)::

    cd drake-distro
    export LD_LIBRARY_PATH=`pwd`/build/install/lib/vtk-5.10:$LD_LIBRARY_PATH
    export PYTHONPATH=`pwd`/build/externals/vtk/Wrapping/Python:`pwd`/build/externals/vtk/bin:$PYTHONPATH

You should now be able to start ``drake-visualizer``.


.. _faq_drake_visualizer_no_module_named_vtk_common_core_python_ros_indigo:

ROS Indigo Users
----------------

To workaround the problem, configure Director's build system to build VTK5 from
source (``drake-visualizer`` is built on Director, which is built on VTK5)::

    cd ~/dev/drake_catkin_workspace/build/drake
    cmake . -DUSE_SYSTEM_VTK=OFF -DWITH_VTK=ON
    cd ~/dev/drake_catkin_workspace
    catkin build

Next, modify two environment variables before starting
``drake-visualizer``::

    export LD_LIBRARY_PATH=$HOME/dev/drake_catkin_workspace/install/lib/vtk-5.10:$LD_LIBRARY_PATH
    export PYTHONPATH=$HOME/dev/drake_catkin_workspace/build/drake/externals/vtk/Wrapping/Python:$HOME/dev/drake_catkin_workspace/build/drake/externals/vtk/bin:$PYTHONPATH

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
