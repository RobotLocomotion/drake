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

Note: This docker image is provided as an experimental feature and is not
presently covered by continuous integration.

.. _docker_getting_started:

Getting Started
===============
Docker containers have emerged as a solution to running code or services in a
way that is isolated from the host operating system. This allows code to be
run/compiled with conflicting dependencies from those of the host operating
system while also isolating these executables from the host file system.
Docker is available for all major operating systems. Please see Docker's
`Getting Started <https://docs.docker.com/get-started/>`_ for basic information
on Docker.

This Dockerfile is based upon the vanilla Ubuntu 16.04 image. Should you want
to build inside of another base image, they are available `here
<https://hub.docker.com/explore/>`_. 

.. _docker_building:

Building
========

Clone Drake source code as described in :ref:`Getting Drake:<getting_drake>`. 

::

  $ cd <drake-distro>

The Dockerfile will copy the <drake-distro> directory into the Docker
container.

::

  $ docker build -t drake -f setup/docker/Dockerfile .

If successful

::

  $ docker images

should show an image named drake.

Note:

::

  $ docker ps

will show any running drake containers on your system.


.. _docker_running:

Running
=======

.. _docker_running_simulation:

Passive Acrobot Simulation
--------------------------

.. _docker_running_simulation_nvidia:

Nvidia drivers:
~~~~~~~~~~~~~~~
The `nvidia-docker <https://github.com/NVIDIA/nvidia-docker/>`_ plugin is
required in order to pass X drawing commands to your host system when the
proprietary Nvidia GPU drivers are installed.


::

  $ xhost +local:root; nvidia-docker run -i --rm -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged -t drake; xhost -local:root

.. _docker_running_simulation_open:

Open source drivers:
~~~~~~~~~~~~~~~~~~~~
::

  $ xhost +local:root; docker run -i --rm -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged -t drake; xhost -local:root

This command will allow access for non-network connections to your local X
server and pass the necessary X11 parameters for graphical display of programs
within the Docker container. The `-i` switch assigns a tty for interactive
text connections within the console. `--rm` will clean up after the image, omit
this to allow the container's file system to persist.
See the `Docker Run Reference
<https://docs.docker.com/engine/reference/run/>`_. For more information on
run options.

It is possible to interactively develop and compile within the Docker container.
Several options exist for retaining code altered or generated within the
Docker image. `Docker cp
<https://docs.docker.com/engine/reference/commandline/cp/>`_ can be used
to copy files out of a running image. 
`-v <https://docs.docker.com/engine/tutorials/dockervolumes/#locate-a-volume>`_
can be used to mount a host directory inside the Docker image. Or you can use
git commands interactively inside the container to push code changes directly
to a repository. 


Note: The --privileged argument is only necessary under security enhanced
linux.

.. _docker_running_shell:

Enter An Interactive Shell
--------------------------

An arbitrary command can be passed to the docker image by appending the
argument to the above commands. Type bash at the end to enter a bash shell in
the docker image.

::

  $ xhost +local:root; docker run -i --rm -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged -t drake bash; xhost -local:root
