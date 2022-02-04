#!/bin/bash

set -eu -o pipefail

readonly BAZEL_VERSION=5.0.0
readonly BAZEL_ROOT=https://github.com/bazelbuild/bazel/releases/download


# Fix ssh permissions.
chmod 700 ~/.ssh
chmod 600 ~/.ssh/known_hosts

# Prepare system to install packages, and apply any updates.
apt-get -y update
apt-get -y upgrade

apt-get -y install lsb-release

# Install prerequisites.
readonly DISTRO=$(lsb_release -sc)

mapfile -t PACKAGES < <(sed -e '/^#/d' < /image/packages-${DISTRO})

apt-get -y install --no-install-recommends ${PACKAGES[@]}

# Install CMake.
# To build vtk-9 on bionic we need an updated CMake.
# See: https://apt.kitware.com/
# TODO(svenevs) Use distro version of CMake when we drop Bionic support.
apt-get -y install --no-install-recommends gpg lsb-release wget
readonly KW_ASC="https://apt.kitware.com/keys/kitware-archive-latest.asc"
wget -O - "$KW_ASC" 2>/dev/null | \
    gpg --dearmor - | \
    tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] " \
    "https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/kitware.list >/dev/null
apt-get -y update
apt-get -y install --no-install-recommends kitware-archive-keyring
apt-get -y install --no-install-recommends \
    "cmake-data=3.16.3-*" "cmake=3.16.3-*"

# Install Bazel.
cd /tmp
wget ${BAZEL_ROOT}/${BAZEL_VERSION}/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
bash /tmp/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
rm /tmp/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
