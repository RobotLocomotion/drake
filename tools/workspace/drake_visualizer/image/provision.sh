#!/bin/bash

set -eu -o pipefail

export DEBIAN_FRONTEND=noninteractive

readonly BAZEL_VERSION=5.1.0
readonly BAZEL_ROOT=https://github.com/bazelbuild/bazel/releases/download

# Install prerequisites.
apt-get -y update
apt-get -y upgrade

xargs -d$'\n' apt-get -y install --no-install-recommends < /image/prereqs

# Install CMake.
# To find the right OpenGL library (GLVND) on bionic we need an updated CMake.
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
apt-get -y install --no-install-recommends unzip
cd /tmp
wget ${BAZEL_ROOT}/${BAZEL_VERSION}/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
bash /tmp/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
rm /tmp/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh

# TODO(svenevs): the manual symlink of /usr/bin/python can be removed when
# bionic support is dropped and the `python-is-python3` package therefore
# available unconditionally.
ln -s /usr/bin/python3 /usr/bin/python
