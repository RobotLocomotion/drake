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
    git cmake ninja-build pkg-config \
    yasm file wget unzip zip ssh

apt-get -y install --no-install-recommends \
    libglib2.0-dev libnlopt-dev \
    libgl1-mesa-dev libxt-dev \
    opencl-headers ocl-icd-opencl-dev

# Install Bazel.
cd /tmp
wget ${BAZEL_ROOT}/${BAZEL_VERSION}/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
bash /tmp/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
rm /tmp/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
