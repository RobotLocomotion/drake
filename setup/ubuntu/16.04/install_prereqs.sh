#!/bin/bash
#
# Prerequisite set-up script for Drake on Ubuntu 16.04.

set -euo pipefail

die () {
    echo "$@" 1>&2
    exit 1
}

me="The Drake prerequisite set-up script"

[[ "${EUID}" -eq 0 ]] || die "${me} must run as root. Please use sudo."

apt update
apt install --no-install-recommends lsb-release wget

[[ "$(lsb_release -sc)" == "xenial" ]] || die "${me} only supports Ubuntu 16.04."

# Install Clang 3.9
while true; do
  echo "The Ubuntu 16.04 distribution includes Clang 3.8 by default."
  echo "To install Clang 3.9 it is necessary to add a Personal Package Archive (PPA)."
  echo "This script will add the repository
    'deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-3.9 main'"
  read -p "Do you want to continue? [Y/n] " yn
  case $yn in
    [Yy]*)
      apt install --no-install-recommends software-properties-common
      wget -q -O - http://llvm.org/apt/llvm-snapshot.gpg.key | apt-key add -
      # In this form, add-apt-repository is only truly idempotent when -s is
      # added, since it otherwise duplicates the commented deb-src line.
      add-apt-repository -s -y "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-3.9 main"
      apt update
      apt install --no-install-recommends clang-3.9 lldb-3.9
      break
      ;;
    [Nn]*) break ;;
    *) echo "Please answer yes or no." ;;
  esac
done

# Install the APT dependencies.
apt update -y
apt install --no-install-recommends $(tr '\n' ' ' <<EOF

bash-completion
binutils
chrpath
clang-4.0
clang-format-4.0
cmake
cmake-curses-gui
coinor-libclp-dev
coinor-libipopt-dev
diffstat
doxygen
g++
g++-5
g++-5-multilib
gcc
gcc-5
gcc-5-multilib
gdb
git
graphviz
libblas-dev
libboost-all-dev
libbz2-dev
libexpat1-dev
libfreetype6-dev
libglib2.0-dev
libglu1-mesa-dev
libhdf5-dev
libjpeg8-dev
libjsoncpp-dev
liblapack-dev
liblz4-dev
libnetcdf-cxx-legacy-dev
libnetcdf-dev
libnlopt-dev
libogg-dev
libpng-dev
libprotobuf-dev
libqt5opengl5-dev
libqt5x11extras5-dev
libtheora-dev
libtiff5-dev
libtinyxml-dev
libtinyxml2-dev
libtool
libxml2-dev
libxt-dev
libyaml-cpp-dev
lldb-4.0
make
mesa-common-dev
openjdk-8-jdk
patchutils
pkg-config
protobuf-compiler
python-dev
python-gtk2
python-lxml
python-matplotlib
python-numpy
python-protobuf
python-pygame
python-scipy
python-sphinx
python-tk
python-yaml
valgrind
zip
zlib1g-dev

EOF
    )

# Install Bazel.
wget -O /tmp/bazel_0.6.1-linux-x86_64.deb https://github.com/bazelbuild/bazel/releases/download/0.6.1/bazel_0.6.1-linux-x86_64.deb
if echo "5012d064a6e95836db899fec0a2ee2209d2726fae4a79b08c8ceb61049a115cd /tmp/bazel_0.6.1-linux-x86_64.deb" | sha256sum -c -; then
  dpkg -i /tmp/bazel_0.6.1-linux-x86_64.deb
else
  die "The Bazel deb does not have the expected SHA256.  Not installing Bazel."
fi

rm /tmp/bazel_0.6.1-linux-x86_64.deb

# Remove deb that we used to generate and install, but no longer need.
if [ -L /usr/lib/ccache/bazel ]; then
  apt purge ccache-bazel-wrapper
fi
