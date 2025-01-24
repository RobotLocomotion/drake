#!/usr/bin/env bash

# Internal script to build dependencies for a Drake wheel.

set -eu -o pipefail

mkdir -p /opt/drake-wheel-build/dependencies/build
cd /opt/drake-wheel-build/dependencies/build

cmake -G Ninja \
    -DCMAKE_INSTALL_PREFIX=/opt/drake-dependencies \
    /opt/drake-wheel-build/dependencies/src

ninja
