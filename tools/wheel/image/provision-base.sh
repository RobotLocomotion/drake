#!/bin/bash

# Internal script to install common (non-Python) build dependencies.
# Docker (Linux) only.

set -eu -o pipefail

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
