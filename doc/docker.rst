.. _docker_entry:

Building Drake in a Docker Container
************************************

.. note::

  These instructions and the Drake Docker images are provided as a courtesy for
  developers and users and are not covered by continuous integration. Support
  is on a best-effort basis.

  However, there are alternative downstream usages of Drake within a Docker
  container:

  * `MIT 6.832 (Underactuated Robotics) Spring 2019 Drake Docker Instructions <http://underactuated.csail.mit.edu/Spring2019/install_drake_docker.html>`_
  * `MIT 6.881 (Intelligent Robot Manipulation) Fall 2018 Drake Docker Instructions <http://manipulation.csail.mit.edu/install_drake_docker.html>`_
  * `Spartan's Docker Build <https://github.com/RobotLocomotion/spartan/blob/master/setup/docker/README.md>`_

.. _installing_docker_and_building_and_running_a_drake_docker_image:

Installing Docker and Building and Running a Drake Docker Image
===============================================================

.. _ubuntu_1604_xenial_xerus:

Ubuntu 16.04 (Xenial Xerus)
---------------------------

Install Docker from the Ubuntu package archive:

::

  sudo apt-get update
  sudo apt-get install --no-install-recommends docker.io
  sudo usermod --append --group docker ${USER}

Reboot or log out and log back in again. Clone the Drake Git repository and
build and run a Drake Docker image:

::

  sudo apt-get install --no-install-recommends git
  git clone https://github.com/RobotLocomotion/drake.git
  cd drake
  docker build --file setup/ubuntu/docker/xenial/Dockerfile --tag drake .
  docker run --interactive --tty drake bash

.. _ubuntu_1604_xenial_xerus_with_proprietary_nvidia_driver_384_and_cuda_75_support:

Ubuntu 16.04 (Xenial Xerus) with Proprietary NVIDIA Driver 384 and CUDA 7.5 Support
-----------------------------------------------------------------------------------

.. warning::

  You MUST first uninstall all previous versions of the proprietary NVIDIA
  driver, CUDA, and the NVIDIA container runtime for Docker. FAILURE TO DO SO
  MAY LEAVE YOUR SYSTEM IN A BROKEN STATE. Please carefully read all
  instructions and proceed with caution. If in doubt, consider following the
  instructions in the :ref:`previous section <ubuntu_1604_xenial_xerus>` to
  install and run Docker without proprietary NVIDIA driver and CUDA support.

Install Docker CE from the Docker package archive:

::

  sudo apt-get update
  sudo apt-get install --no-install-recommends \
    apt-transport-https \
    ca-certificates \
    gnupg \
    passwd
  sudo apt-key adv --fetch-keys https://download.docker.com/linux/ubuntu/gpg
  echo 'deb [arch=amd64] https://download.docker.com/linux/ubuntu xenial stable' \
    | sudo tee /etc/apt/sources.list.d/docker.list
  sudo apt-get update
  sudo apt-get install --no-install-recommends docker-ce
  sudo usermod --append --group docker ${USER}

Refer to the warning at the beginning of this section and uninstall all
previous versions of the proprietary NVIDIA driver,
`CUDA <https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#handle-uninstallation>`_,
and the `NVIDIA container runtime for Docker <https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-2.0)#removing-nvidia-docker-10>`_.

Install the proprietary NVIDIA driver 384 and CUDA toolkit 7.5 from the Ubuntu
package archive:

::

  sudo apt-get install --no-install-recommends nvidia-384 nvidia-cuda-toolkit

Install the NVIDIA container runtime 2.0 for Docker from the NVIDIA package
archive:

::

  sudo apt-get install wget
  sudo apt-key adv --fetch-keys https://nvidia.github.io/nvidia-docker/gpgkey
  wget -O - https://nvidia.github.io/nvidia-docker/ubuntu16.04/nvidia-docker.list \
    | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
  sudo apt-get update
  sudo apt-get install --no-install-recommends nvidia-docker2
  sudo pkill -SIGHUP dockerd

Reboot or log out and log back in again. Clone the Drake Git repository and
build and run a Drake Docker image:

::

  sudo apt-get install --no-install-recommends git
  git clone https://github.com/RobotLocomotion/drake.git
  cd drake
  docker build \
    --file setup/ubuntu/docker/xenial/Dockerfile.nvidia-cuda-7.5-devel-ubuntu16.04 \
    --tag drake .
  docker run --interactive --runtime=nvidia --tty drake bash

.. _ubuntu_1804_bionic_beaver:

Ubuntu 18.04 (Bionic Beaver)
----------------------------

Install Docker from the Ubuntu package archive:

::

  sudo apt-get update
  sudo apt-get install --no-install-recommends docker.io
  sudo usermod --append --group docker ${USER}

Reboot or log out and log back in again. Clone the Drake Git repository and
build and run a Drake Docker image:

::

  sudo apt-get install --no-install-recommends git
  git clone https://github.com/RobotLocomotion/drake.git
  cd drake
  docker build --file setup/ubuntu/docker/bionic/Dockerfile --tag drake .
  docker run --interactive --tty drake bash

.. _ubuntu_1804_bionic_beaver_with_proprietary_nvidia_driver_390_and_cuda_91_support:

Ubuntu 18.04 (Bionic Beaver) with Proprietary NVIDIA Driver 390 and CUDA 9.1 Support
------------------------------------------------------------------------------------

.. warning::

  You MUST first uninstall all previous versions of the proprietary NVIDIA
  driver, CUDA, and the NVIDIA container runtime for Docker. FAILURE TO DO SO
  MAY LEAVE YOUR SYSTEM IN A BROKEN STATE. Please carefully read all
  instructions and proceed with caution. If in doubt, consider following the
  instructions in the :ref:`previous section <ubuntu_1804_bionic_beaver>` to
  install and run Docker without proprietary NVIDIA driver and CUDA support.

Install Docker CE from the Docker package archive:

::

  sudo apt-get update
  sudo apt-get install --no-install-recommends \
    apt-transport-https \
    ca-certificates \
    gnupg \
    passwd
  sudo apt-key adv --fetch-keys https://download.docker.com/linux/ubuntu/gpg
  echo 'deb [arch=amd64] https://download.docker.com/linux/ubuntu bionic stable' \
    | sudo tee /etc/apt/sources.list.d/docker.list
  sudo apt-get update
  sudo apt-get install --no-install-recommends docker-ce
  sudo usermod --append --group docker ${USER}

Refer to the warning at the beginning of this section and uninstall all
previous versions of the proprietary NVIDIA driver,
`CUDA <https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#handle-uninstallation>`_,
and the `NVIDIA container runtime for Docker <https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-2.0)#removing-nvidia-docker-10>`_.

Install the proprietary NVIDIA driver 390 and CUDA toolkit 9.1 from the Ubuntu
package archive:

::

  sudo apt-get install --no-install-recommends \
    nvidia-cuda-toolkit \
    nvidia-driver-390

Install the NVIDIA container runtime 2.0 for Docker from the NVIDIA package
archive:

::

  sudo apt-get install wget
  sudo apt-key adv --fetch-keys https://nvidia.github.io/nvidia-docker/gpgkey
  wget -O - https://nvidia.github.io/nvidia-docker/ubuntu18.04/nvidia-docker.list \
    | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
  sudo apt-get update
  sudo apt-get install --no-install-recommends nvidia-docker2
  sudo pkill -SIGHUP dockerd

Reboot or log out and log back in again. Clone the Drake Git repository and
build and run a Drake Docker image:

::

  sudo apt-get install --no-install-recommends git
  git clone https://github.com/RobotLocomotion/drake.git
  cd drake
  docker build \
    --file setup/ubuntu/docker/bionic/Dockerfile.nvidia-cuda-9.1-devel-ubuntu18.04 \
    --tag drake .
  docker run --interactive --runtime=nvidia --tty drake bash

.. _other_platforms:

Other Platforms
---------------

Follow the instructions on the Docker website to
`install stable Docker Community Edition <https://docs.docker.com/install/>`_.

Clone the Drake Git repository and build and run a Drake Docker image:

::

  git clone https://github.com/RobotLocomotion/drake.git
  cd drake
  docker build --file setup/ubuntu/docker/bionic/Dockerfile --tag drake .
  docker run --interactive --tty drake bash

.. _running_a_drake_docker_image_with_graphical_interface_support:

Running a Drake Docker Image with Graphical Interface Support
=============================================================

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

``xhost +local:root`` and ``xhost -local:root`` allow and remove access for
non-network connections to your local X server and pass the necessary X11
parameters for the graphical display of programs within the Docker container.

Use ``--runtime=nvidia`` if you built your image with proprietary NVIDIA driver
and CUDA support:

::

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

The default command will start ``drake-visualizer`` and run the bowling ball
simulation.

.. _useful_docker_documentation:

Useful Docker Documentation
===========================

* `docker build command reference <https://docs.docker.com/engine/reference/commandline/build/>`_
  (building an image from a Dockerfile)
* `docker cp command reference <https://docs.docker.com/engine/reference/commandline/cp/>`_
  (copying files and/or folders between a container and the local filesystem)
* `docker run command reference <https://docs.docker.com/engine/reference/commandline/run/>`_
  (running a command in a new container)
* `docker volumes guide <https://docs.docker.com/storage/volumes/>`_
  (persisting data generated by and/or used by Docker containers)
