.. _docker_entry:

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

The Nvidia Dockerfile is based upon the nvidia docker plugin base image, which 
also contains CUDA support. The opensource Dockerfile is based on the 
the vanilla Ubuntu 16.04 image and is intended to support opensource graphics
card drivers such as nouveau and intel. Should you need to build inside of
another base image (FROM line in the Dockerfile), they are available `here
<https://hub.docker.com/explore/>`_. 

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

These steps on Ubuntu 16.04 are:

::

  $ sudo apt-get update; sudo apt-get install docker.io
  $ sudo systemctl enable docker
  $ sudo systemctl start docker
  $ sudo usermod -aG docker <username>

Log out and then back in.

.. _docker_building:

Building
========

Clone the Drake source code as described in :ref:`Getting Drake<getting_drake>`. 

The the following build commands will copy the full <drake-distro> directory
into the Docker container where it may be built and run.

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

should show an image named drake and 

::

  $ docker ps

will show any running drake containers on your system.

.. _docker_running:

Running
=======

.. _docker_running_simulation:

The simplest run command is

::

  $ docker run -it drake bash

which will give you bash shell access to the Ubuntu 16.04 Docker container
where you can run commands such as:

::

  $ cd /drake-distro
  $ bazel build //...
  $ bazel test //...

These commands will build all packages using bazel and run all tests.

Graphical Interface
~~~~~~~~~~~~~~~~~~~

The run command in order to get graphical interfaces from the Docker container
is a bit more involved. Two systems are described below one with Nvidia
proprietary graphics card drivers and one with open source drivers like Nouveau
and Intel.

.. _docker_running_simulation_nvidia:

Nvidia drivers:
---------------
The `nvidia-docker <https://github.com/NVIDIA/nvidia-docker/>`_ plugin is
required in order to pass Xorg drawing commands to your host system when the
proprietary Nvidia GPU drivers are installed.

Note: on Ubuntu 16.04 use the following nvidia-docker build recipe

::
  git clone https://github.com/NVIDIA/nvidia-docker.git
  cd nvidia-docker/
  make
  sudo PREFIX=/usr/local/bin make install
  sudo nvidia-docker volume setup
  nvidia-docker run --rm nvidia/cuda nvidia-smi

because the .deb package methods assume a much more recent Docker.


::

  $ xhost +local:root; nvidia-docker run -i --rm -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged -t drake; xhost -local:root

The default command defined behavior will start the Drake visualizer and run 
the bowling ball simulation.

Walking through this command::

  $ xhost +local
  
will allow access for non-network connections to your local X server and pass
the necessary X11 parameters for graphical display of programs within the Docker
container.::

  docker-nvidia

is an Nvidia plugin that couples with the proprietary Nvidia drivers and gives
access to advanced features like CUDA.

The::

  -i

switch assigns a tty for interactive text connections within
the console.::

  --rm

will clean up after the image, omit this to allow the container's file system to
persist.::

  -e DISPLAY

Forwards your host DISPLAY environment variable to the Docker container.::

  -e QT_X11_NO_MITSHM=1

specifies to not use the MIT magic cookie.::

  -v /tmp/.X11-unix:/tmp/.X11-unix

shares the host .X11 interface with the Docker container as a volume.::

  --privileged

is only needed on selinux systems.::

  -t drake

provides the Docker container name and::

  $ xhost -local:root

removes the permission given earlier for local non-network connections to X.


See the `Docker Run Reference
<https://docs.docker.com/engine/reference/run/>`_. For more information on
run options.

It is also possible to enter a bash shell for interactive development with:

::

  $ xhost +local:root; nvidia-docker run -i --rm -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged -t drake bash; xhost -local:root

.. _docker_running_simulation_open:

Open source drivers:
~~~~~~~~~~~~~~~~~~~~
With open source graphics drivers like Nouveau and Intel you do not need the
nvidia-docker plugin.

::

  $ xhost +local:root; docker run -i --rm -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged -t drake; xhost -local:root


Sharing Files Between Host and Docker:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

It is possible to interactively develop and compile within the Docker container.
Several options exist for retaining code altered or generated within the
Docker image. `Docker cp
<https://docs.docker.com/engine/reference/commandline/cp/>`_ can be used
to copy files into and out of a running image. 
`-v <https://docs.docker.com/engine/tutorials/dockervolumes/#locate-a-volume>`_
can be used to mount a host directory inside the Docker image at the expense
of file system isolation. Or you can use git commands interactively inside the
container to push code changes directly to a repository. 
