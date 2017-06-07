.. _docker:

********************************
Building Drake in a Docker Image
********************************

.. _docker_intro:

Introduction
============
There are two Docker containers provided that can build Drake in isolated
Ubuntu 16.04 environments. The intention is to make it easy develop for and 
use drake on a variety of host operating systems.

.. _docker_getting_started:

Getting Started
===============
Docker containers have emerged as a solution to running code or services in a
way that is isolated from the host operating system. This allows code to be
run/compiled with conflicting dependencies from those of the host operating
system while also isolating these executables from the host filesystem.
Docker is available for all major operating systems. Please see Docker's
`Getting Started <https://docs.docker.com/get-started/>`_ for basic information
on Docker.

This Dockerfile is based upon the vanilla Ubuntu 16.04 image. Should you want
to build inside of another base image, they are available `here
<https://hub.docker.com/explore/>`_. 

.. _docker_building:

Building
========
::

  $ cd <drake-distro>

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
the docker image.

::

  $ xhost +local:root; docker run -ti --rm -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged drake bash; xhost -local:root
