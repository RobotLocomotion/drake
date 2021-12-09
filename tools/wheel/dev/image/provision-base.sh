#!/bin/bash

set -e

BAZEL_VERSION=4.2.1
BAZEL_ROOT=https://github.com/bazelbuild/bazel/releases/download

# Fix ssh permissions.
chmod 700 ~/.ssh
chmod 600 ~/.ssh/known_hosts

# Install prerequisites.
apt-get -y update
apt-get -y upgrade

apt-get -y install --no-install-recommends \
    default-jdk \
    autoconf automake \
    libtool libltdl-dev \
    gcc g++ gfortran libgfortran-7-dev \
    libclang-9-dev clang-format-9 \
    git ninja-build pkg-config \
    yasm file wget unzip zip ssh

apt-get -y install --no-install-recommends \
    libglib2.0-dev libnlopt-dev \
    libgl1-mesa-dev libxt-dev \
    opencl-headers ocl-icd-opencl-dev

# To build vtk-9 on bionic we need an updated CMake.
# See: https://apt.kitware.com/
# TODO(svenevs) Revert this when we drop support for bionic
apt-get -y install --no-install-recommends gpg lsb-release
kw_asc="https://apt.kitware.com/keys/kitware-archive-latest.asc"
wget -O - "$kw_asc" 2>/dev/null | \
    gpg --dearmor - | \
    tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] " \
    "https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/kitware.list >/dev/null
apt-get -y update
rm /usr/share/keyrings/kitware-archive-keyring.gpg
apt-get -y install --no-install-recommends kitware-archive-keyring
apt-get -y install --no-install-recommends \
    "cmake-data=3.16.3-*" "cmake=3.16.3-*"

# Install Bazel.
cd /tmp
wget ${BAZEL_ROOT}/${BAZEL_VERSION}/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
bash /tmp/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
rm /tmp/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
