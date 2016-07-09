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

Once OS and ROS is installed, install
`Catkin Tools <http://catkin-tools.readthedocs.io/en/latest/>`_ by following
the instructions
`here <http://catkin-tools.readthedocs.io/en/latest/installing.html>`_.
We use ``catkin_tools`` instead of the normal ``catkin_build`` because Drake
isn't a *real* Catkin package meaning it needs to built in a nisolated manner
from the rest of the packages in the ROS workspace.

.. _drake_catkin_create_workspace_directories:

Step 1: Create Directories for Holding a ROS Catkin Workspace
=============================================================

This assumes you want to create a *new* ROS workspace
in ``~/dev`` called ``drake_catkin_workspace``. The commands below can be
customized with a different workspace name and location.

If you already have a ROS Catkin workspace and simply want to add Drake as a
package within it, you can skip this step and go straight to
:ref:`_drake_catkin_add_repos`.

Execute the following commands to create that directory structure that will hold
the ROS workspace::

    mkdir -p ~/dev/drake_catkin_workspace/src

.. _drake_catkin_add_repos:

Step 2: Add Clones of ``drake`` and ``drake_ros_integration`` to the Workspace
==============================================================================

Add local clones of the ``drake`` and ``drake_ros_integration`` repositories
to your workspace::

    cd ~/dev/drake_catkin_workspace/src
    git clone git@github.com:RobotLocomotion/drake.git
    git clone git@github.com:liangfok/drake_ros_integration.git


.. _drake_catkin_build_workspace:

Step 3: Build the Workspace
===========================

Execute the following commands to build the workspace::

    cd ~/dev/drake_catkin_workspace/
    source /opt/ros/indigo/setup.bash
    catkin init
    catkin build

There are numerous optional flags that can be included after the ``catkin build``
command listed above. For example, one flag is ``-DDISABLE_MATLAB=TRUE`, which
disables MATLAB support, which may be useful if you have MATLAB installed but
don't have access to a license server. There are many additional command line
flags that, for example, enables support for certain optimizers like
`SNOPT <http://www.sbsi-sol-optimize.com/asp/sol_product_snopt.htm>`_.
For a full list of these optional command line flags, see the variables defined
in ``~/dev/drake_catkin_workspace/src/drake/CMakeLists.txt``.

Note also that Catkin by default performs a multi-threaded build.
If your computer does not have sufficient computational resources to support
this, you can add a ``-j1`` flag after the ``catkin_make`` command to force a
single-threaded build, which uses fewer resources.

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

.. _drake_catkin_run_car_example:

Running An Example: Car Simulation
----------------------------------

To run Drake's ROS-powered cars example, execute::

    .. TODO(liang.fok) Combine the roslaunch and rosrun commands into one!
    cd ~/dev/drake_catkin_workspace
    source devel/setup.bash
    roslaunch drake_cars_examples rviz_prius.launch
    rosrun drake_cars_examples car_sim_lcm_and_ros src/drake/drake/examples/Cars/models/prius/prius_with_lidar.sdf src/drake/drake/examples/Cars/models/stata_garage_p1.sdf

