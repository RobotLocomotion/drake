.. _docker_entry:

Building Drake in a Docker Container
************************************

.. _docker_intro:

Introduction
============
Docker containers have emerged as a solution to running code or services in a
way that is isolated from the host operating system. This allows code to be
compiled and run on systems running unsupported operating systems or with
incompatible configurations to the software dependencies. Docker is available
for `all major operating systems <https://www.docker.com/community-edition>`_.

Several Docker containers are provided with Drake to allow developers to test
and develop without needing to configure a supported operating system. These
containers will build Drake in isolated Ubuntu environments.

The NVIDIA Dockerfiles is based upon the nvdia-docker base image, which
also contains CUDA support. The open source Dockerfiles are based on the
the offical Ubuntu base images and are intended to support open source graphics
card drivers such as Nouveau and Intel. Should you need to build inside of
another base image (FROM line in the Dockerfile), they are available `here
<https://hub.docker.com/explore/>`_.

.. note::

  The Drake Docker images are provided as an experimental feature and is not
  presently covered by continuous integration.

  However, there are downstream usages of Drake within a Docker container:

  * `MIT 6.832 (Underactuated Robotics) Drake Docker Instructions <http://underactuated.csail.mit.edu/Spring2019/install_drake_docker.html>`_,
    used by many students across all nooks and crannies of Windows, Mac, Linux.
  * `MIT 6.881 (Intelligent Robot Manipulation) Drake Docker Instructions <http://manipulation.csail.mit.edu/install_drake_docker.html>`_,
  * `Spartan's Docker Build <https://github.com/RobotLocomotion/spartan/blob/master/setup/docker/README.md>`_

.. _docker_getting_started:

Installing Docker Community Edition
===================================
Follow the instructions on the Docker website to
`install stable Docker Community Edition <https://docs.docker.com/install/>`_.

These steps on Ubuntu 18.04 (Bionic Beaver) are as follows:

::

  sudo apt-get update
  sudo apt-get install --no-install-recommends apt-transport-https ca-certificates gnupg
  sudo apt-key adv --fetch-keys https://download.docker.com/linux/ubuntu/gpg
  echo 'deb [arch=amd64] https://download.docker.com/linux/ubuntu bionic stable' | sudo tee /etc/apt/sources.list.d/docker.list
  sudo apt-get update
  sudo apt-get install --no-install-recommends containerd.io docker-ce docker-ce-cli
  sudo usermod --append --groups docker $USER

Restart or log out and then back in to complete installation

Verify that Docker Community Edition is installed correctly by running the
``hello-world`` image.

::

  docker run hello-world

You should see an informational message that includes the following text:

::

  Hello from Docker!
  This message shows that your installation appears to be working correctly.

.. _installing_nvidia_docker_20:

Installing nvidia-docker 2.0
============================
If you have an NVIDIA GPU and are running a recent version of Linux, optionally
follow the instructions on the NVIDIA website to
`install NVIDIA CUDA drivers and toolkit <https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html>`_.

These steps on Ubuntu 18.04 (Bionic Beaver) are as follows:

::

  sudo apt-get update
  sudo apt-get install --no-install-recommends build-essential linux-headers-$(uname -r) wget
  wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-repo-ubuntu1804_10.1.105-1_amd64.deb
  sudo dpkg -i cuda-repo-ubuntu1804_10.1.105-1_amd64.deb
  rm -f cuda-repo-ubuntu1804_10.1.105-1_amd64.deb
  sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
  sudo apt-get update
  sudo apt-get install --no-install-recommends cuda-10-1

Verify that the NVIDIA drivers are installed correctly by running the
``nvidia-smi`` utility. You should see an informational message that includes
the NVIDIA driver and CUDA versions and the type of GPU that is installed:

::

  +-----------------------------------------------------------------------------+
  | NVIDIA-SMI 418.39       Driver Version: 418.39       CUDA Version: 10.1     |
  |-------------------------------+----------------------+----------------------+
  | GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
  | Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
  |===============================+======================+======================|
  |   0  Tesla M60           On   | 00000000:00:1E.0 Off |                    0 |
  | N/A   29C    P8    23W / 150W |      0MiB /  7618MiB |      0%      Default |
  +-------------------------------+----------------------+----------------------+


Then, follow the instructions on the nvidia-docker wiki to
`install nvidia-docker 2.0 <https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-2.0)>`_.

These steps on Ubuntu 18.04 (Bionic Beaver) are as follows:

::

  wget -O - https://nvidia.github.io/nvidia-docker/ubuntu18.04/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
  sudo apt-key adv --fetch-keys https://nvidia.github.io/nvidia-docker/gpgkey
  sudo apt-get update
  sudo apt-get --no-install-recommends install nvidia-docker2
  sudo pkill -SIGHUP dockerd

Verify that nvidia-docker 2.0 is installed correctly by running the
``nvidia/cuda`` image.

::

  docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi

You should see an informational message from ``nvidia-smi`` running inside a
Docker container, similar to that seen when previously verifying that the
NVIDIA drivers were installed correctly.

.. _docker_building:

Building a Drake Docker Image
=============================

Install ``git`` and clone the Drake source code as described in
:ref:`Getting Drake <getting_drake>`.

These steps on Ubuntu 18.04 (Bionic Beaver) are as follows:

::

  sudo apt-get update
  sudo apt-get install --no-install-recommends git
  git clone https://github.com/RobotLocomotion/drake.git

Change into the root of the cloned repository and build a drake image from one
of the provided Dockerfiles as follows:

::

  cd drake
  docker build --file <dockerfile> --tag drake .

where ``<dockerfile>`` is one of the following files:

* ``setup/ubuntu/docker/xenial/Dockerfile``: Ubuntu 16.04 (Xenial Xerus) base
  image
* ``setup/ubuntu/docker/xenial/Dockerfile.nvidia-cuda-10.1-devel-ubuntu16.04``:
  Ubuntu 16.04 (Xenial Xerus) base image with nvidia-docker support
* ``setup/ubuntu/docker/bionic/Dockerfile``: Ubuntu 18.04 (Bionic Beaver) base
  image
* ``setup/ubuntu/docker/bionic/Dockerfile.nvidia-cuda-10.1-devel-ubuntu18.04``:
  Ubuntu 18.04 (Bionic Beaver) base image with nvidia-docker support

Verify that image has been build successfully using the ``docker images``
command. You should see an informational message similar to the following:

::

  REPOSITORY          TAG                      IMAGE ID            CREATED             SIZE
  drake               latest                   000000000000        About a minute ago  6.45GB

.. _docker_running:

Running a Drake Docker Image
============================

The simplest run commands are:

::

  docker run --interactive --tty drake bash
  docker run --interactive --runtime=nvidia --tty drake bash

Use ``--runtime=nvidia`` if you built your image with nvidia-docker support.
These commands will give you bash shell access to a running Docker container
where you can run commands such as:

::

  bazel build //...
  bazel test //...

These commands will build all packages using bazel and run all tests.

Graphical Interface
~~~~~~~~~~~~~~~~~~~

The run command in order to get graphical interfaces from the Docker container
is a bit more involved:

::

  xhost +local:root; docker run \
    --env=DISPLAY \
    --env=QT_X11_NO_MITSHM=1 \
    --interactive \
    --ipc=host \
    --privileged \
    --tty \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
    drake; xhost -local:root

  xhost +local:root; docker run \
    --env=DISPLAY \
    --env=QT_X11_NO_MITSHM=1 \
    --interactive \
    --ipc=host \
    --privileged \
    --runtime=nvidia \
    --tty \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
    drake; xhost -local:root

As above, use ``--runtime=nvidia`` if you built your image with nvidia-docker
support. The default command defined behavior will start the Drake Visualizer
and run the bowling ball simulation.

Walking through these commands:

* ``xhost +local:root`` will allow access for non-network connections to your
  local X server and pass the necessary X11 parameters for graphical display of
  programs within the Docker container.
* `` --env=DISPLAY`` forwards your host DISPLAY environment variable to the Docker
  container.
* ``--env=QT_X11_NO_MITSHM=1 --ipc=host`` specifies to not use the MIT magic cookie.
* ``--interactive --tty`` assigns a pseudo-TTY for interactive text connections within the console.
* ``--privileged`` is only needed on SELinux systems.
* ``--runtime=nvidia`` is the nvidia-docker runtime.
* ``--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw`` shares the host .X11 interface with the
  Docker container as a volume.
* ``drake`` provides the Docker image name.
* ``xhost -local:root`` removes the permission given earlier for local
  non-network connections to X.

See the `docker run reference
<https://docs.docker.com/engine/reference/commandline/run/>`_ for more information on
run options.

It is also possible to enter a bash shell for interactive development with:

::

  xhost +local:root; docker run \
    --env=DISPLAY \
    --env=QT_X11_NO_MITSHM=1 \
    --interactive \
    --ipc=host \
    --privileged \
    --tty \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
    drake bash; xhost -local:root

  xhost +local:root; docker run \
    --env=DISPLAY \
    --env=QT_X11_NO_MITSHM=1 \
    --interactive \
    --ipc=host \
    --privileged \
    --runtime=nvidia \
    --tty \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
    drake bash; xhost -local:root

where you may want to try various demonstrations, e.g.:

::

  bazel run //examples/contact_model:bowling_ball
  bazel run //examples/kuka_iiwa_arm:kuka_simulation
  bazel run //examples/kuka_iiwa_arm/dev/monolithic_pick_and_place:monolithic_pick_and_place_demo


Note: these are currently not rendering properly due to VTK .obj/.mtl importing.

Sharing Files Between Host and Docker:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

It is possible to interactively develop and compile within the Docker container.
Several options exist for retaining code altered or generated within the
Docker image:

* `docker cp <https://docs.docker.com/engine/reference/commandline/cp/>`_ can
  be used to copy files into and out of a running image.
* `-v, --volume <https://docs.docker.com/storage/volumes/#choose-the--v-or-mount-flag>`_
  can be used to mount a host directory inside the Docker image at the expense
  of file system isolation. Or you can use git commands interactively inside the
  container to push code changes directly to a repository.
