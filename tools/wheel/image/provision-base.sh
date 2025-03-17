#!/bin/bash

# Internal script to install common (non-Python) build dependencies.
# Docker (Linux) only.

set -eu -o pipefail

# Prepare system to install packages, and apply any updates.
apt-get -y update
apt-get -y upgrade

apt-get -y install lsb-release

# Install prerequisites.
readonly DISTRO=$(lsb_release -sc)

mapfile -t PACKAGES < <(sed -r -e '/^(#|$)/d' < /image/packages-${DISTRO})

apt-get -y install --no-install-recommends ${PACKAGES[@]}
