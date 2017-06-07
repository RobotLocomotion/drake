.. _docker:

********************************
Building Drake in a Docker Image
********************************

.. contents:: `Table of contents`
   :depth: 3
   :local:

.. _docker_intro:

Introduction
============
There are two Docker containers provided that can build Drake in isolated
Ubuntu 16.04 environments. The intention is to make it easy develop for and 
use drake on a variety of host operating systems.

.. _docker_building:

Building
========
::
  $ cd <drake-root-dir>

If you have the Nvidia drivers installed:::
  $ docker build -t drake -f setup/docker/Dockerfile.nvidia .

If you are using open source graphics drivers (Nouveau, Intel, ...):::
  $ docker build -t drake -f setup/docker/Dockerfile.opensource .

.. _docker_running:

Running
=======

.. _docker_running_simulation:

Passive Acrobot Simulation
--------------------------

.. _docker_running_simulation_nvidia:

Nvidia drivers:  (requires nvidia-docker plugin)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
::
  $ xhost +local:root; nvidia-docker run -ti --rm -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged drake; xhost -local:root

.. _docker_running_simulation_open:

Open source drivers:
~~~~~~~~~~~~~~~~~~~~
::
  $ xhost +local:root; docker run -ti --rm -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged drake; xhost -local:root

Note: The --privileged argument is only necessary under security enhanced
linux.

.. _docker_running_shell:

Enter An Interactive Shell
--------------------------
An arbitrary command can be passed to the docker image by appending the
argument to the above commands. Type bash at the end to enter a bash shell in
the docker image.::
  $ xhost +local:root; docker run -ti --rm -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged drake bash; xhost -local:root

