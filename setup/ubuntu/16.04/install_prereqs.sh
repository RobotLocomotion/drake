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
# TODO(david-german-tri): Reorganize these by the external that requires them.

SCM_TOOLS=(git subversion)
COMPILERS=(g++-5 clang default-jdk)
DRAKE_BUILD_TOOLS=(cmake cmake-curses-gui ninja-build make pkg-config)
EXTERNAL_BUILD_TOOLS=(autoconf automake libtool swig unzip)
EXTERNAL_COMPILERS=(g++-5-multilib gfortran)
PERL_DEPS=(perl libhtml-form-perl libwww-perl libterm-readkey-perl)
VTK_DEPS=(libvtk-java libvtk5-dev libvtk5-qt4-dev)
QT_DEPS=(libqt4-dev libqt4-opengl-dev libqwt-dev)
DOCS_DEPS=(doxygen graphviz)
PYTHON_DEPS=(python python-bs4 python-dev python-gtk2 python-html5lib python-numpy python-sphinx python-vtk)
MISC_DEPS=(libgtk2.0-dev libmpfr-dev libpng12-dev bison flex)
DEBUG_TOOLS=(valgrind gdb)

apt install --assume-yes \
${SCM_TOOLS[*]} \
${COMPILERS[*]} \
${DRAKE_BUILD_TOOLS[*]} \
${EXTERNAL_BUILD_TOOLS[*]} \
${EXTERNAL_COMPILERS[*]} \
${PERL_DEPS[*]} \
${MISC_DEPS[*]} \
${VTK_DEPS[*]} \
${QT_DEPS[*]} \
${DOCS_DEPS[*]} \
${PYTHON_DEPS[*]} \
${DEBUG_TOOLS[*]}

# TODO(david-german-tri): Do we need to munge the MATLAB C++ libraries?
# http://drake.mit.edu/ubuntu.html#matlab

