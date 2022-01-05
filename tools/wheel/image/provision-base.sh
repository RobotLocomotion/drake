#!/bin/bash

set -eu -o pipefail

readonly BAZEL_VERSION=4.2.1
readonly BAZEL_ROOT=https://github.com/bazelbuild/bazel/releases/download

readonly DISTRO=$(grep UBUNTU_CODENAME /etc/os-release | \
                  cut -f2 -d= | tr -d '"')

# Fix ssh permissions.
chmod 700 ~/.ssh
chmod 600 ~/.ssh/known_hosts

# Install prerequisites.
apt-get -y update
apt-get -y upgrade

mapfile -t PACKAGES < \
    <(cat /image/packages-common /image/packages-${DISTRO} | sed -e '/^#/d')

apt-get -y install --no-install-recommends ${PACKAGES[@]}

# Install Bazel.
cd /tmp
wget ${BAZEL_ROOT}/${BAZEL_VERSION}/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
bash /tmp/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
rm /tmp/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
