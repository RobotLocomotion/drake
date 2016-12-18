.. _build_from_source_using_ros_catkin:

**************************************
Building Drake as a ROS Catkin Package
**************************************

.. _build_from_source_using_ros_catkin_introduction:

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


Drake can be configured to be a package within a ROS Catkin workspace.
Instructions for how to do this are given below.

.. _build_from_source_using_ros_catkin_instructions:

Instructions
============

Please select the configuration that matches your system. If you don't find a
match, that simply means it hasn't been tested. You might still be able to get
Drake + ROS to work, though it may require some additional effort customizing
the commands mentioned in the instructions that are linked to below.

.. toctree::
    :maxdepth: 1

    from_source_ros_indigo
    from_source_ros_kinetic

.. _drake_ros_additional_notes:

Additional Notes
================

.. _drake_ros_additional_build_flags:

Additional Build Flags
----------------------

There are numerous optional build flags that can be specified with the
``catkin config`` command. These options can also be passed directly to
``catkin build``, though they will not persist. For a full list, execute::

    catkin config --help

The default build type documented in the build instructions is
``RelWithDebInfo``. Alternatives include ``Release`` and ``Debug``. Another
``cmake`` flag is ``-DDISABLE_MATLAB=TRUE``, which disables MATLAB support. This
may be useful if you have MATLAB installed but don't have access to a license
server. There are many additional command line flags that, for example, enables
support for certain optimizers like
`SNOPT <http://www.sbsi-sol-optimize.com/asp/sol_product_snopt.htm>`_.
For a full list of these optional command line flags, see the variables defined
in ``~/dev/drake_catkin_workspace/src/drake/cmake/options.cmake``.

Note also that Catkin by default performs a multi-threaded build. If your
computer does not have sufficient computational resources to support this, you
can add a ``-j1`` flag after the ``catkin config`` command to force a
single-threaded build, which uses fewer resources.

.. _drake_ros_build_documenation:

Building Drake's Documentation
------------------------------

To build Drake's documentation, execute::

    cd ~/dev/drake_catkin_workspace/build/drake/drake
    make documentation

The documentation will be located in
``~/dev/drake_catkin_workspace/build/drake/drake/doc``.

.. _drake_ros_ci_documenation:

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

.. _drake_ros_clean_build:

How to Perform a Clean Build
----------------------------

To do a clean build (also colloquially referred to as a "comprehensive nuke" by
the Drake development community), execute::

    cd ~/dev/drake_catkin_workspace
    catkin clean
    cd src/drake
    rm -rf externals
    git reset --hard HEAD
    git clean -fdx
    cd ~/dev/drake_catkin_workspace
    source /opt/ros/[distro]/setup.bash
    catkin config --cmake-args -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo
    catkin build

Be sure to replace ``[distro]`` above with your actual ROS distribution.

.. _drake_ros_run_unit_tests:

How to Run Unit Tests
=====================

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
