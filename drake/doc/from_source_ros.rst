.. _build_from_source_using_ros_catkin:

**************************************
Building Drake as a ROS Catkin Package
**************************************

.. _drake_catkin_instruction:

Introduction
============

The `Robot Operating System (ROS) <http://www.ros.org/>`_ provides a software
platform for conducting advanced robotics research and development. Since its
`creation in the mid 2000s <http://www.ros.org/history/>`_, ROS has
attained a `tremendous following <http://wiki.ros.org/Metrics>`_ due in part to
its integration of many infrastructure-level components that are important to
robotics and the increased collaborative potential resulting from platform
standardization. It is arguably among the most successful open source robotics
platforms to date in terms of fostering collaboration, research, and innovation
within robotics.


Drake can be configured to be a package within a ROS workspace. Instructions for
how to do this are given below. Before starting, ensure
`ROS is installed <http://wiki.ros.org/ROS/Installation>`_
and your public SSH key is
`added to your github account <https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/>`_.

.. _drake_catkin_prerequisites:

Step 0: Install Prerequisites
=============================

Install
`Ubuntu 14.04.4 LTS (Trusty Tahr) <http://releases.ubuntu.com/14.04/>`_ and
`ROS Indigo <http://wiki.ros.org/indigo>`_. Other versions of the OS and ROS
may or may not work.

Install package ``ros-indigo-ackermann-msgs``, which is used by
software in `drake_ros_systems <https://github.com/RobotLocomotion/drake/tree/master/ros/drake_ros_systems>`_::

    sudo apt-get install ros-indigo-ackermann-msgs

Once the OS and ROS are installed, install
`Catkin Tools <http://catkin-tools.readthedocs.io/en/latest/>`_ by following
the instructions
`here <http://catkin-tools.readthedocs.io/en/latest/installing.html>`_.
We use ``catkin_tools`` due to its better support building pure CMake packages
alongside Catkin packages.

.. _drake_catkin_create_workspace_directories:

Step 1: Create Directories for Holding a ROS Catkin Workspace
=============================================================

This assumes you want to create a *new* ROS workspace
in ``~/dev`` called ``drake_catkin_workspace``. The commands below can be
customized with a different workspace name and location.

If you already have a ROS Catkin workspace and simply want to add Drake as a
package within it, you can skip this step and go straight to
:ref:`drake_catkin_add_repos`.

Execute the following commands to create a directory structure for holding the
ROS workspace::

    mkdir -p ~/dev/drake_catkin_workspace/src

.. _drake_catkin_add_repos:

Step 2: Add ``drake`` and ``drake_ros_integration`` to the Workspace
====================================================================

Add ``drake`` and ``drake_ros_integration`` to the workspace::

    cd ~/dev/drake_catkin_workspace/src
    git clone git@github.com:RobotLocomotion/drake.git
    ln -s drake/ros drake_ros_integration

Note that ``drake_ros_integration`` is a symbolic link. This allows us to keep
everything in Drake's main repository without needing to completely reorganize
the files in it.

.. _drake_catkin_build_workspace:

Step 3: Build the Workspace
===========================

Execute the following commands to build the workspace::

    cd ~/dev/drake_catkin_workspace
    source /opt/ros/indigo/setup.bash
    catkin init
    catkin config --cmake-args -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo
    catkin build

There are numerous optional build flags that can be specified with the
``catkin config`` command listed above. These options can also be passed
directly to ``catkin build``, though they will not persist.
For a full list, execute::

    catkin config --help

The example above includes the ``cmake`` flag that specifies a build type of
``RelWithDebInfo``. Alternatives include ``Release`` and ``Debug``. Another
``cmake`` flag is ``-DDISABLE_MATLAB=TRUE``, which
disables MATLAB support. This may be useful if you have MATLAB installed but
don't have access to a license server. There are many additional command line
flags that, for example, enables support for certain optimizers like
`SNOPT <http://www.sbsi-sol-optimize.com/asp/sol_product_snopt.htm>`_.
For a full list of these optional command line flags, see the variables defined
in ``~/dev/drake_catkin_workspace/src/drake/CMakeLists.txt``.

Note also that Catkin by default performs a multi-threaded build.
If your computer does not have sufficient computational resources to support
this, you can add a ``-j1`` flag after the ``catkin config`` command to force a
single-threaded build, which uses fewer resources.

Later, if you want to do a clean build, you can execute::

    cd ~/dev/drake_catkin_workspace
    catkin clean
    cd src/drake
    rm -rf externals
    git reset --hard HEAD
    git clean -fdx
    cd ~/dev/drake_catkin_workspace
    source /opt/ros/indigo/setup.bash
    catkin build

.. _drake_catkin_run_unit_tests:

Step 4: Run Unit Tests
======================

Execute the following commands to run Drake's ROS-based unit tests::

    cd ~/dev/drake_catkin_workspace
    source devel/setup.bash
    catkin build --verbose --no-deps drake_ros_systems --make-args run_tests

To run a single unit test like ``ros_test.test`` within package
``drake_ros_systems``, execute::

    rostest drake_ros_systems ros_test.test

To run Drake's non-ROS-based unit tests, execute::

    cd ~/dev/drake_catkin_workspace/build/drake/drake
    ctest

For more information about how to run Drake's non-ROS unit tests, see
:ref:`unit-test-instructions`.

.. _drake_catkin_additional_notes:

Additional Notes
================

.. _drake_catkin_build_documenation:

Building Drake's Documentation
------------------------------

To build Drake's documentation, execute::

    cd ~/dev/drake_catkin_workspace/build/drake/drake
    make documentation

The documentation will be located in
``~/dev/drake_catkin_workspace/build/drake/drake/doc``.

.. _drake_catkin_ci_documenation:

Scheduling a Drake / ROS Continuous Integration Test
----------------------------------------------------

Drake's Jenkin's Continuous Integration (CI) pre-merge test matrix currently
does not include a Drake + ROS column. Thus, if you change Drake's source
code and want to know whether it breaks the Drake + ROS integration, you must
manually schedule a test by posting the following comment in your PR::

    @drake-jenkins-bot linux-gcc-experimental-ros please

The command above will schedule a Drake + ROS CI pre-merge test called
"`linux-gcc-experimental-ros`". As indicated by its name, this uses the `gcc`
compiler. Links to the results are available on the PR's web page and from here:
https://drake-jenkins.csail.mit.edu/view/Experimental/job/linux-gcc-experimental-ros/.

To test the Drake + ROS integration using the `clang` compiler, post the
following comment in your PR::

    @drake-jenkins-bot linux-clang-experimental-ros please

The comment above will schedule a test called "`linux-clang-experimental-ros`".
Links to the results are available on the PR's web page and here:
https://drake-jenkins.csail.mit.edu/view/Experimental/job/linux-clang-experimental-ros/.

To schedule a full test of Drake + ROS + MATLAB with `gcc`, post the following
comment on your PR::

    @drake-jenkins-bot linux-gcc-experimental-matlab-ros please

The results will be available here:
https://drake-jenkins.csail.mit.edu/view/Experimental/job/linux-gcc-experimental-matlab-ros/.
