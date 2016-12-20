.. _build_from_source_using_ros_kinetic:

***************************************************************************
Building Drake as a Catkin Package in ROS Kinetic and Ubuntu 16.04 (Xenial)
***************************************************************************

.. _drake_ros_kinetic_prerequisites:

Step 1: Install Prerequisites
=============================

Install `Ubuntu 16.04 LTS (Xenial Xerus) <http://releases.ubuntu.com/16.04/>`_
and `ROS Kinetic <http://wiki.ros.org/kinetic>`_. We recommend installing the
"desktop-full" version of ROS to get its full feature set.

Add your public SSH key to your github.com account by following
`these instructions <https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/>`_. This is necessary because the
instructions below assume you can clone Git repositories from github.com using
SSH rather than HTTPS.

Install package ``ros-kinetic-ackermann-msgs``::

    sudo apt-get install ros-kinetic-ackermann-msgs

Once the OS and ROS are installed, install
`Catkin Tools <http://catkin-tools.readthedocs.io/en/latest/>`_ by following
the instructions
`here <http://catkin-tools.readthedocs.io/en/latest/installing.html>`_.
We use ``catkin_tools`` due to its better support building pure CMake packages
alongside Catkin packages.

.. _drake_ros_kinetic_create_workspace_directories:

Step 2: Create Directories for Holding a ROS Catkin Workspace
=============================================================

This assumes you want to create a *new* ROS workspace in ``~/dev`` called
``drake_catkin_workspace``. The commands below can be customized with a
different workspace name and location. If you already have a ROS Catkin
workspace and simply want to add Drake as a package within it, you can skip this
step and go straight to :ref:`drake_ros_kinetic_add_repos`.

Execute the following commands to create a directory structure for holding the
ROS workspace::

    mkdir -p ~/dev/drake_catkin_workspace/src

.. _drake_ros_kinetic_add_repos:

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
`Drake issue #4445 <https://github.com/RobotLocomotion/drake/issues/4445>`_).

.. _drake_ros_kinetic_install_drake_dependencies:

Step 4: Install Drake's Dependencies
====================================

Drake includes a convenient Ubuntu 16.04 shell script that installs all of
its dependencies. This script is located in
``~/dev/drake_catkin_workspace/src/drake/setup/ubuntu/16.04/install_prereqs.sh``.

There is currently a conflict between Drake's need for ``libvtk5-dev`` and
ROS Kinetic's need for ``libvtk6-dev``. To resolve this issue, we do the
following:

Open
``~/dev/drake_catkin_workspace/src/drake/setup/ubuntu/16.04/install_prereqs.sh``
and delete the following lines::

    libvtk-java
    libvtk5-dev
    libvtk5-qt4-dev
    python-vtk

Then execute the script::

    cd ~/dev/drake_catkin_workspace/src/drake/setup/ubuntu/16.04
    sudo ./install_prereqs.sh

.. _drake_ros_kinetic_build_workspace:

Step 5: Build the Workspace
===========================

Execute the following commands to build the workspace::

    cd ~/dev/drake_catkin_workspace
    source /opt/ros/kinetic/setup.bash
    catkin init
    catkin config --cmake-args -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo
    catkin build

The build will fail because ``libvtk5-dev`` is not installed. To get around
this, execute the following commands, which configure Director to obtain VTK5
by building it from source::

    cd ~/dev/drake_catkin_workspace/build/drake/externals/director
    cmake . -DUSE_SYSTEM_VTK=OFF
    cd ~/dev/drake_catkin_workspace
    catkin build

.. _drake_ros_kinetic_environment_variables:

Step 6: Modify Environment Variables to Allow VTK5 Libraries to be Found
========================================================================

Try launching ``drake-visualizer``::

    cd ~/dev/drake_catkin_workspace
    ./install/bin/drake-visualizer

You may encounter the following error::

    ./install/bin/drake-visualizer: error while loading shared libraries: libvtkPythonCore.so.5.10: cannot open shared object file: No such file or directory

To fix this error, modify your ``LD_LIBRARY_PATH`` and ``PYTHON_PATH``
environment variables as follows::


    export LD_LIBRARY_PATH=$HOME/dev/drake_catkin_workspace/install/lib/vtk-5.10:$LD_LIBRARY_PATH
    export PYTHONPATH=$HOME/dev/drake_catkin_workspace/build/drake/externals/director/src/vtk-build/Wrapping/Python:/home/liang/dev/drake_catkin_workspace/build/drake/externals/director/src/vtk-build/bin:$PYTHONPATH

You should now be able to run ``drake-visualizer``. For more background
information, see
`this thread <https://github.com/RobotLocomotion/drake/issues/3703#issuecomment-252236733>`_.

This concludes the instructions for how to install and build Drake using ROS
Kinetic on Ubuntu 16.04. See
:ref:`these additional notes <drake_ros_additional_notes>` on where to proceed
from here.
