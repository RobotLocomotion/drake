.. _build_from_source_using_ros_indigo:

**************************************************************************
Building Drake as a Catkin Package in ROS Indigo and Ubuntu 14.04 (Trusty)
**************************************************************************

.. _drake_ros_indigo_prerequisites:

Step 1: Install Prerequisites
=============================

Install `Ubuntu 14.04.4 LTS (Trusty Tahr) <http://releases.ubuntu.com/14.04/>`_
and `ROS Indigo <http://wiki.ros.org/indigo>`_. We recommend installing the
"desktop-full" version of ROS to get its full feature set.

Add your public SSH key to your github.com account by following
`these instructions <https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/>`_. This is necessary because the
instructions below assume you can clone Git repositories from github.com using
SSH rather than HTTPS.

Install package ``ros-indigo-ackermann-msgs``::

    sudo apt-get install ros-indigo-ackermann-msgs

Once the OS and ROS are installed, install
`Catkin Tools <http://catkin-tools.readthedocs.io/en/latest/>`_ by following
the instructions
`here <http://catkin-tools.readthedocs.io/en/latest/installing.html>`_.
We use ``catkin_tools`` due to its better support building pure CMake packages
alongside Catkin packages.

.. _drake_ros_indigo_create_workspace_directories:

Step 2: Create Directories for Holding a ROS Catkin Workspace
=============================================================

This assumes you want to create a *new* ROS workspace in ``~/dev`` called
``drake_catkin_workspace``. The commands below can be customized with a
different workspace name and location. If you already have a ROS Catkin
workspace and simply want to add Drake as a package within it, you can skip this
step and go straight to :ref:`drake_ros_indigo_add_repos`.

Execute the following commands to create a directory structure for holding the
ROS workspace::

    mkdir -p ~/dev/drake_catkin_workspace/src

.. _drake_ros_indigo_add_repos:

Step 3: Add ``drake`` and ``drake_ros_integration`` to the Workspace
====================================================================

Add ``drake`` and ``drake_ros_integration`` to the workspace::

    cd ~/dev/drake_catkin_workspace/src
    git clone git@github.com:RobotLocomotion/drake.git
    git remote set-url origin git@github.com:[your github user name]/drake.git
    git remote add upstream git@github.com:RobotLocomotion/drake.git
    git remote set-url --push upstream no_push
    ln -s drake/ros drake_ros_integration

Note that ``drake_ros_integration`` is a symbolic link. This allows us to keep
everything in Drake's main repository without needing to reorganize the files in
it (see
`Drake issue #4445 <https://github.com/RobotLocomotion/drake/issues/4445>`).

.. _drake_ros_indigo_install_drake_dependencies:

Step 4: Install Drake's Dependencies
====================================

Follow the instructions :ref:`here <build_from_source_trusty>` to install
Drake's dependencies on Ubuntu 14.04.

.. _drake_ros_indigo_build_workspace:

Step 5: Build the Workspace
===========================

Execute the following commands to build the workspace::

    cd ~/dev/drake_catkin_workspace
    source /opt/ros/indigo/setup.bash
    catkin init
    catkin config --cmake-args -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo
    catkin build

This concludes the instructions for how to install and build Drake using ROS
Indigo on Ubuntu 14.04. See
:ref:`these additional notes <drake_ros_additional_notes>` on where to proceed
from here.
