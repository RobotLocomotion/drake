#!/bin/bash
#
# Install development and runtime prerequisites for binary distributions of
# Drake on Ubuntu 16.04.

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo 'This script must be run as root' >&2
  exit 1
fi

apt update
apt install --no-install-recommends wget

wget -O - https://drake-apt.csail.mit.edu/drake.pub.gpg | apt-key add
echo 'deb [arch=amd64] https://drake-apt.csail.mit.edu xenial main' > /etc/apt/sources.list.d/drake.list

apt update
apt install --no-install-recommends $(tr '\n' ' ' <<EOF
build-essential
cmake
coinor-libipopt1v5
dreal=4.18.01.3
libblas3
libboost-all-dev
libexpat1
libgflags-dev
libgl1-mesa-glx
libglib2.0-0
libhdf5-10
libibex-dev=2.6.5.20180123154310.gitf618c7b296182f90a84d54936d144b87df0747b9~16.04
libjpeg8
libjsoncpp1
liblapack3
libnetcdf-c++4
libnetcdf11
libnlopt0
libogg0
libpng12-0
libprotobuf-dev
libqt5multimedia5
libqt5x11extras5
libtheora0
libtiff5
libtinyxml2-dev
libtinyxml2.6.2v5
libxml2
libxt6
libyaml-cpp0.5v5
openjdk-8-jre
python-lxml
python-numpy
python-scipy
python-yaml
qtbase5-dev
zlib1g
EOF
)
