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

# Install Clang 3.9
while true; do
  echo "The Ubuntu 16.04 distribution includes Clang 3.8 by default."
  echo "To install Clang 3.9 it is necessary to add a Personal Package Archive (PPA)."
  echo "This script will add the repository
    'deb http://llvm.org/apt/xenial/ llvm-toolchain-xenial-3.9 main'"
  read -p "Do you want to continue? [Y/n] " yn
  case $yn in
    [Yy]*)
      apt-get install --no-install-recommends lsb-core software-properties-common wget
      wget -q -O - http://llvm.org/apt/llvm-snapshot.gpg.key | sudo apt-key add -
      add-apt-repository -y "deb http://llvm.org/apt/xenial/ llvm-toolchain-xenial-3.9 main"
      apt-get update
      apt install --no-install-recommends clang-3.9
      break
      ;;
    [Nn]*) break ;;
    *) echo "Please answer yes or no." ;;
  esac
done

# The CI scripts require a newer version of CMake than apt installs.
# Only install CMake if it's not installed or older than 3.5.
install_cmake=true
if command -v cmake &>/dev/null; then
  cmake_version=$(cmake --version) &>/dev/null
  cmake_version=${cmake_version:14:3}
  if dpkg --compare-versions $cmake_version ge 3.5; then
    echo "CMake is already installed ($cmake_version)"
    install_cmake=false
  fi
fi
if $install_cmake; then
  apt install --no-install-recommends cmake
  apt install --no-install-recommends cmake-curses-gui
fi

# Install the APT dependencies.
# TODO(david-german-tri): Can we remove libvtk-java?
apt install --no-install-recommends $(tr '\n' ' ' <<EOF

autoconf
automake
bison
default-jdk
doxygen
flex
freeglut3-dev
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
python-yaml
unzip
valgrind

EOF
    )

# TODO(david-german-tri): Do we need to munge the MATLAB C++ libraries?
# http://drake.mit.edu/ubuntu.html#matlab

