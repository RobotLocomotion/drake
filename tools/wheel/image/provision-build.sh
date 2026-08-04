#!/bin/bash

# Internal script to prepare the build location.
#
# This creates a unique directory under ``$HOME/.drake-wheel-build`` which is
# used to contain all artifacts produced and/or consumed during a wheel build.
# This directory is symlinked as ``/tmp/drake-wheel-build`` in order to provide
# a consistent location for the build process. Both the directory itself and
# the symlink are normally removed after a successful build, unless the user
# specified ``--keep-build``.
#
# An optional argument serves as a prefix for the created directory, which can
# help disambiguate retained builds on macOS (which performs builds on the host
# system rather than in a container).
#
# On macOS, this script is also used to create a unique space for running
# tests. A second optional argument may be used to override the ``build``
# portion of the ``/tmp/drake-wheel-build`` symlink.

set -eu -o pipefail

mkdir -p ~/.drake-wheel-build
readonly d="$(mktemp -d ~/.drake-wheel-build/${1:-}XXXXXXXXXX)"
ln -nsf "${d}" /tmp/drake-wheel-${2:-build}
