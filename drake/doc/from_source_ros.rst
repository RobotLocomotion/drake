.. _build_from_source_using_ros_catkin:

**************************************
Building Drake as a ROS Catkin Package
**************************************

.. _drake_ros_catkin_introduction:

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

The following instructions are known to work on
`Ubuntu 14.04.4 LTS (Trusty Tahr) <http://releases.ubuntu.com/14.04/>`_ and
`ROS Indigo <http://wiki.ros.org/indigo>`_. Other versions of the OS and ROS
may or may not work. They also assume you want to create a *new* ROS workspace
in ``~/dev`` called ``drake_catkin_workspace``. The commands below can be
customized with a different workspace name and location.

If you already have a ROS Catkin workspace and simply want to add Drake as a
package within it, you can skip :ref:`create_catkin_workspace` and proceed to
:ref:`add_and_compile_drake_to_ros_workspace`.

.. _create_catkin_workspace:

Step 1: Create a ROS Catkin Workspace
=====================================

To create a ROS Catkin workspace, open a terminal and execute the following
commands::

    mkdir -p ~/dev/drake_catkin_workspace/src
    cd ~/dev/drake_catkin_workspace/src
    source /opt/ros/indigo/setup.bash
    catkin_init_workspace
    cd ..
    catkin_make

.. _add_and_compile_drake_to_ros_workspace:

Step 2: Add Drake to the ROS Catkin Workspace and Re-Build the Workspace
========================================================================

To add Drake to your ROS Catkin workspace, execute the following commands (note:
you may need to customize the commands below if you elected to place the
workspace in a different location)::

    cd ~/dev/drake_catkin_workspace/src
    git clone git@github.com:RobotLocomotion/drake.git
    cd ..
    catkin_make

There are numerous optional flags that can be included after the ``catkin_make``
command listed above. For example, one flag is ``-DDISABLE_MATLAB=TRUE``, which
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
