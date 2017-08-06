.. _build_from_source:

**********************************
Source installation (OS X, Ubuntu)
**********************************

Optional: Setting up the MATLAB Compiler
========================================

Make sure that the MATLAB executable is in your path.  (e.g., typing ``matlab``
at the system command line should start an instance of MATLAB).  For example,
on Mac you might consider
``sudo ln -s /Applications/MATLAB_R2016a.app/bin/matlab /usr/local/bin/matlab``
, or you can actually add the MATLAB/bin directory to your system path.

.. _getting_drake:

Getting Drake
=============

We recommend that you `setup SSH access to Github.com <https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/>`_
to avoid needing to type your password each time you access it. The following
instructions assume you have uploaded your public SSH key to your Github
account.

Now run::

    git clone git@github.com:RobotLocomotion/drake.git drake-distro


Note: the build process may encounter problems if you have unusual characters
like parentheses in the absolute path to the drake-distro directory
(see `#394 <https://github.com/RobotLocomotion/drake/issues/394>`_).

The above ``git clone`` command will configure Drake's primary repository as a
remote called ``origin``. We recommend that you configure your fork of Drake's
primary repository as the ``origin`` remote and Drake's primary repository as
the ``upstream`` remote. This can be done by executing the following commands::

    cd drake-distro
    git remote set-url origin git@github.com:[your github user name]/drake.git
    git remote add upstream git@github.com:RobotLocomotion/drake.git
    git remote set-url --push upstream no_push

.. _platform_specific_setup:

Mandatory platform specific instructions
========================================

Before running the build, you must follow some one-time platform-specific
setup steps:

.. toctree::
    :maxdepth: 1

    mac
    ubuntu_trusty
    ubuntu_xenial

See :ref:`supported configurations <supported-configurations>`
for the configurations and platforms that Drake officially supports.
All else being equal, we would recommend developers use Ubuntu Xenial.

.. _build_the_collection:

Build the collection
====================
There are four ways to build Drake:

1. :ref:`Using Bazel <build_with_bazel>`
2. :ref:`Using Make <build_with_make>`
3. :ref:`Using Ninja <build_with_ninja>`
4. :ref:`Using ROS Catkin <build_with_ros_catkin>`

For instructions on how to switch build systems, see
:ref:`this subsection <identifying_build_system_used>`.

Bazel is the build system used by most Drake Developers.  It should be your
first choice if you intend to stay up-to-date with the latest changes,
contribute new features, or experiment directly with Drake.  It is also the
least complex choice to get started.

Alternatively, if you want to use Drake as a toolbox within another project,
you might instead use
:ref:`make <build_with_make>`,
:ref:`ninja <build_with_ninja>`, or
:ref:`catkin <build_with_ros_catkin>`
as detailed below.

.. _build_with_bazel:

Build with Bazel
----------------

For instructions, jump to :ref:`Using Bazel <using_bazel>`, or check out the
full details at:

.. toctree::
    :maxdepth: 1

    bazel

The rest of this page is irrelevant if you are using Bazel.

.. _build_with_make:

Build with Make
---------------
First, confirm that `Make <https://www.gnu.org/software/make/>`_ is installed
on your system by executing::

    make --version

To build with ``Make``, execute::

    cd drake-distro
    mkdir build
    cd build
    cmake ..
    make

**Do NOT use sudo.** Just ``make`` is sufficient, and will prevent problems
later. Feel free to use ``make -j`` if your platform supports it. Note that the
above ``cmake`` command does not specify a build type, so drake will be built
with ``Release`` by default. If you wish to build with a different build type,
change the cmake command to ``cmake .. -DCMAKE_BUILD_TYPE:STRING=Debug``
to build with build type "Debug". Alternative build modes include
"RelWithDebInfo" and "Release". They differ in terms of the amount of debug
symbols included in the resulting binaries and how efficiently the code
executes.

In addition to the build mode, Drake has many other build options. You can view
them in ``drake-distro/CMakeLists.txt``. The build options typically start with
"WITH\_". The options can be included in the above ``cmake`` command by adding
additional ``-D`` flags. You can also specify them using a text-based GUI by
executing::

    cd drake-distro/build
    ccmake ..

To get details about the actual compiler and linker commands, execute::

    cd drake-distro/build
    make VERBOSE=true

.. _build_with_ninja:

Build with Ninja
----------------
First, confirm that `Ninja <https://ninja-build.org/>`_ is installed
on your system::

    ninja --version

Drake uses `CMake <https://cmake.org/>`_ to generate Ninja files within an
out-of-source build directory. You can configure CMake options by passing
them at the ``cmake`` command line with ``-D``, or in a GUI by running
``ccmake`` instead of ``cmake``. For instance, the following sequence of
commands generates Ninja files, and then runs the Ninja build.

::

    cd drake-distro
    mkdir build
    cd build
    cmake .. -G Ninja
    ninja

Ninja can rebuild Drake from within ``drake-distro/build/drake/`` without
rebuilding the entire super-build.  It can also build specific targets.
Tab-completion is supported.

::

    cd drake-distro/build/drake
    ninja

To review the raw shell commands, compiler flags, and linker flags that CMake
generated, consult ``build.ninja`` and ``drake/build.ninja``, or run
``ninja -v`` for a verbose build.

.. _build_with_ros_catkin:

Build with ROS Catkin
---------------------

See:

.. toctree::
    :maxdepth: 1

    from_source_ros

.. _build_and_install_directories:

Locations of Build and Install Directories
==========================================

This section contains information about where Drake's build artifacts are
located. They assume you are following the official from-source build
instructions given above. The locations will differ if you're building Drake
using an alternative method, e.g., completely out of source.

Builds based on `Make <https://www.gnu.org/software/make/>`_ and
`Ninja <https://ninja-build.org/>`_ place build artifacts in
``drake-distro/build/``. Externally visible build artifacts are placed in
``drake-distro/build/install``.

Builds using ROS Catkin place build artifacts in Catkin's
`development space <http://wiki.ros.org/catkin/workspaces#Development_.28Devel.29_Space>`_
and place externally-visible build artifacts in Catkin's
`install space <http://wiki.ros.org/catkin/workspaces#Install_Space>`_. When
following the
:ref:`official Drake/ROS installation instructions <build_from_source_using_ros_catkin>`,
these spaces are typically ``drake_catkin_workspace/devel`` and
``drake_catkin_workspace/install``, respectively.

.. _identifying_build_system_used:

Identifying the Build System Used
=================================

If you encounter an existing from-source installation of Drake and aren't sure
whether it was built using ``Make`` or ``Ninja``, [1]_ look
in ``drake-distro/build``. If there is a file called ``Makefile``, Drake was
built using ``Make``. If there is a file called ``rules.ninja``, Drake was built
using ``Ninja``.

.. [1] ROS Catkin is not listed since you should immediately know if Drake was :ref:`compiled using ROS Catkin <build_from_source_using_ros_catkin>` based on whether it is located in a ROS workspace. Once compiled using Catkin, do not attempt to switch build systems.

.. _how_to_switch_build_systems:

How to Switch Build Systems
===========================

To switch between using ``make`` vs. ``ninja``, first
:ref:`clean your workspace <how_to_clean_your_workspace>`. Then follow
the :ref:`original build instructions <build_the_collection>` using the desired
build system.

.. _how_to_clean_your_workspace:

How to Clean Your Workspace
===========================

First save, commit, and push all of your work. The following commands
*are destructive*.

If you're using ``make`` or ``ninja``, execute::

    cd drake-distro
    rm -rf build
    rm -rf externals
    git clean -fdx
    git reset --hard HEAD

If you're using ROS Catkin, simply execute::

    cd drake_catkin_workspace
    catkin clean

.. _test_from_source_installation:

Test Your Installation
======================

To run some unit tests, execute the following::

    cd drake-distro/build/drake
    ctest -VV

For more details on how to run Drake's unit tests, see the instructions
here: :ref:`unit-test-instructions`.

If you have problems, please check the :doc:`faq`.  If the solution is not
there, or if you discover something missing from our installation instructions
or lists of prerequisites, then please
`file an issue <https://github.com/RobotLocomotion/drake/issues/new>`_ and label
it as *installation*.

Testing Your MATLAB Installation
--------------------------------

Start MATLAB, then at the MATLAB prompt do::

    cd drake-distro/drake/matlab
    addpath_drake
    cd solvers/test
    testMathematicalProgram

You should not see any error messages.

For more details, see :ref:`Using Drake from Matlab <matlab-bindings>`.

Stay up to date
===============


To work on the bleeding edge, do::

    cd drake-distro
    git checkout master
    git pull upstream master
    cd build
    make

This is especially useful if you are ready to contribute your work back to the
main repository with a
`pull request <https://help.github.com/articles/using-pull-requests/>`_.

Troubleshooting
===============

If you're unable to launch ``drake-visualizer`` due to a
"No module named vtkCommonCorePython" error, see
:ref:`this FAQ <faq_drake_visualizer_no_module_named_vtk_common_core_python_non_ros>`.
