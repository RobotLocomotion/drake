#!/usr/bin/env bash

# Internal script to build dependencies for a Drake wheel.

set -eu -o pipefail

mkdir -p ${DRAKE_WHEELBUILD_PREFIX:-}/dependencies/build
cd ${DRAKE_WHEELBUILD_PREFIX:-}/dependencies/build

cmake -G Ninja \
    -DCMAKE_INSTALL_PREFIX=/opt/drake-dependencies \
    ${DRAKE_WHEELBUILD_PREFIX:-}/dependencies/src

ninja

if [ "$(uname)" == "Linux" ]; then
    ln -s /opt/drake-dependencies/lib/pkgconfig /usr/local/lib
    ln -s /opt/drake-dependencies/share/pkgconfig /usr/local/share

    ln -s /opt/drake-dependencies/bin/patchelf /usr/local/bin/patchelf
fi
