#!/usr/bin/env bash

# Internal script to build dependencies for a Drake wheel.

set -eu -o pipefail

mkdir -p /opt/drake-wheel-build/dependencies/build
cd /opt/drake-wheel-build/dependencies/build

cmake -G Ninja \
    -DCMAKE_INSTALL_PREFIX=/opt/drake-dependencies \
    /opt/drake-wheel-build/dependencies/src

ninja

if [[ "$(uname)" == "Linux" ]]; then
    ln -s /opt/drake-dependencies/bin/patchelf /usr/local/bin/patchelf

    # Libraries we get from the distro that get bundled into the wheel need to
    # have their licenses bundled also.
    mkdir -p /opt/drake-dependencies/licenses/mumps
    cp -t /opt/drake-dependencies/licenses/mumps \
        /usr/share/doc/libmumps-seq-dev/copyright
fi
