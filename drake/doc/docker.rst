.. _docker:

Building Drake in a Docker Container
************************************

.. _docker_intro:

Introduction
============
Docker containers have emerged as a solution to running code or services in a
way that is isolated from the host operating system. This allows code to be
compiled and run on systems running unsupported operating systems or with
incompatable configurations to the software dependencies. Docker is available
for `all major operating systems <https://www.docker.com/community-edition>`_.

Two Docker containers are provided with Drake to allow developers to test and
develop without needing to configure a supported operating system. These
containers will build Drake in isolated Ubuntu 16.04 environments.

Note: This docker image is provided as an experimental feature and is not
presently covered by continuous integration.

.. _docker_getting_started:

Getting Started
===============
In order to get docker installed, please follow guides specific to your
operating system. The typical steps are:

  #. Install docker from your distribution's package manager or official installer
     from `Docker
     <https://store.docker.com/search?type=edition&offering=community>`_

  #. Enable Docker to run as a daemon/service
  #. Add appropriate users to the docker group to give permissions to interact
     with the Docker service

  #. Log out and back in to update user groups
  #. Happy dockering!

The Nvidia Dockerfile is based upon the nvidia docker plugin base image, which 
also contains CUDA support, where the opensource Dockerfile is based on the 
the vanilla Ubuntu 16.04 image and is intended to support opensource graphics
card drivers such as nouveau and intel. Should you need to build inside of
another base image, they are available `here
<https://hub.docker.com/explore/>`_. 


.. _docker_building:

Building
========

Clone Drake source code as described in :ref:`Getting Drake:<getting_drake>`. 

The Dockerfile will copy the <drake-distro> directory into the Docker
container.

Nvidia
~~~~~~
When using the Nvidia proprietary drivers:

::

  $ cd <drake-distro>
  $ docker build -t drake -f setup/docker/Dockerfile.ubuntu16.04.nvidia .

Open Source
~~~~~~~~~~~
When using open source video drivers (nouveau, intel, ...):

::

  $ cd <drake-distro>
  $ docker build -t drake -f setup/docker/Dockerfile.ubuntu16.04.opensource .

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
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. _docker_running_simulation_nvidia:

Nvidia drivers:
---------------
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
