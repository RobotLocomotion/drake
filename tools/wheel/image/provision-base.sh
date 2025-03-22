#!/bin/bash

# Internal script to install common (non-Python) build dependencies.
# Docker (Linux) only.

set -eu -o pipefail

# Enable CRB and EPEL.
dnf -y install --setopt=install_weak_deps=False \
    dnf-plugins-core \
    epel-release
dnf config-manager --set-enabled crb
dnf config-manager --set-enabled epel

# Ensure base system is up to date.
dnf -y upgrade

# Get list of required packages
mapfile -t PACKAGES < <(sed -r -e '/^(#|$)/d' < /image/packages-almalinux)

# Install prerequisites.
dnf -y install --setopt=install_weak_deps=False ${PACKAGES[@]}
