#!/bin/bash
# Prerequisite set-up script for Drake on Ubuntu 16.04.
# 16.04 support is in beta. It is not tested in CI or officially supported.

if [[ $EUID -ne 0 ]]; then
   echo "The Drake prerequisite set-up script must run as root. Please use sudo." 1>&2
   exit 1
fi

. /etc/lsb-release

if [[ $DISTRIB_RELEASE -ne "16.04" ]]; then
  echo "The Drake prerequisite set-up script only supports Ubuntu 16.04." 1>&2
  exit 1
fi

# Install the APT dependencies.
# TODO(david-german-tri): Can we remove libvtk-java, subversion?
apt install --no-install-recommends \
autoconf \
automake \
bison \
clang \
cmake \
cmake-curses-gui \
default-jdk \
doxygen \
flex \
g++-5 \
g++-5-multilib \
gdb \
gfortran-5 \
git \
graphviz \
libgtk2.0-dev \
libhtml-form-perl \
libmpfr-dev \
libpng12-dev \
libqt4-dev \
libqt4-opengl-dev \
libqwt-dev \
libterm-readkey-perl \
libtool \
libvtk5-dev \
libvtk5-qt4-dev \
libvtk-java \
libwww-perl \
make \
ninja-build \
perl \
pkg-config \
python-bs4 \
python-dev \
python-gtk2 \
python-html5lib \
python-numpy \
python-sphinx \
python-vtk \
subversion \
swig \
unzip \
valgrind

# TODO(david-german-tri): Do we need to munge the MATLAB C++ libraries?
# http://drake.mit.edu/ubuntu.html#matlab

