#!/bin/bash
# Prerequisite set-up script for Drake on Ubuntu 16.04.
# 16.04 support is in beta. It is not tested in CI or officially supported.

set -eu

die () {
    echo "$@" 1>&2
    exit 1
}

me="The Drake prerequisite set-up script"

[[ $EUID -eq 0 ]] || die "$me must run as root. Please use sudo."

. /etc/lsb-release

[[ $DISTRIB_RELEASE == "16.04" ]] || die "$me only supports Ubuntu 16.04."

# Install the APT dependencies.
# TODO(david-german-tri): Can we remove libvtk-java, subversion?
apt install --no-install-recommends $(tr '\n' ' ' <<EOF

autoconf
automake
bison
clang
cmake
cmake-curses-gui
default-jdk
doxygen
flex
g++-5
g++-5-multilib
gdb
gfortran
gfortran-5
git
graphviz
libgtk2.0-dev
libhtml-form-perl
libmpfr-dev
libpng12-dev
libqt4-dev
libqt4-opengl-dev
libqwt-dev
libterm-readkey-perl
libtool
libvtk-java
libvtk5-dev
libvtk5-qt4-dev
libwww-perl
libxmu-dev
make
ninja-build
perl
pkg-config
python-bs4
python-dev
python-gtk2
python-html5lib
python-numpy
python-sphinx
python-vtk
subversion
swig
unzip
valgrind

EOF
    )

# TODO(david-german-tri): Do we need to munge the MATLAB C++ libraries?
# http://drake.mit.edu/ubuntu.html#matlab

