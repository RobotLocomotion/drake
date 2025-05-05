#!/bin/bash

# Internal script to prepare the build location.
#
# This procedure first creates the super root of the wheel build at
# ``$HOME/.drake-wheel-build``. Then, the script creates a unique temporary
# build directory for each wheel built as a subdirectory of the super root.
# Finally, the temporary directory is symlinked to ``/tmp/drake-wheel-build``
# for consistent reference in later steps of the wheel building process.
#
# The script takes an optional command-line argument denoting the python
# version of the wheel being built. This adds the specified version as a
# prefix to the random build directory name. This way, if the ``--keep-build``
# flag is set for a macOS build, the user can differentiate between wheel
# builds by python version (this is not applicable to Ubuntu builds since
# Linux wheels are built in Docker containers).

set -eu -o pipefail

mkdir -p ~/.drake-wheel-build
readonly d="$(mktemp -d ~/.drake-wheel-build/${1:-}XXXXXXXXXX)"
ln -nsf "${d}" /tmp/drake-wheel-build
