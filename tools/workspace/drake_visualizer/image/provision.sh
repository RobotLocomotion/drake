#!/bin/bash

set -eu -o pipefail

export DEBIAN_FRONTEND=noninteractive

# Install prerequisites.
apt-get -y update
apt-get -y upgrade

xargs -d$'\n' apt-get -y install --no-install-recommends < /image/prereqs

rm -rf /var/lib/apt/lists/*

# Install CMake and Bazel.
# Note that we need a more recent version than is available on Bionic to ensure
# that we get the updated FindOpenGL and therefore link the correct flavor of
# the OpenGL library (GLVND). With the version of CMake from the Ubuntu Bionic
# distribution, we would need even more drastic patching of drake-visualizer.
# TODO(svenevs) Use distro version of CMake when we drop Bionic support.
export APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1

apt-key --keyring /etc/apt/trusted.gpg.d/bazel.gpg adv --quiet \
    --fetch-keys https://bazel.build/bazel-release.pub.gpg
apt-key --keyring /etc/apt/trusted.gpg.d/kitware.gpg adv --quiet \
    --fetch-keys https://apt.kitware.com/keys/kitware-archive-latest.asc
echo 'deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8' \
    > /etc/apt/sources.list.d/bazel.list
echo "deb https://apt.kitware.com/ubuntu $(lsb_release --codename --short) main" \
    > /etc/apt/sources.list.d/kitware.list

apt-get -y update
apt-get -y install --no-install-recommends bazel cmake

rm -rf /var/lib/apt/lists/*

# Note that we symlink /usr/bin/python since the package names to install an
# executable named as such are not the same in different Ubuntu packages.
ln -s /usr/bin/python3 /usr/bin/python
