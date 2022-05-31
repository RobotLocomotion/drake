#!/bin/bash

set -eu -o pipefail

mkdir -p /dependencies/build
cd /dependencies/build

cmake -G Ninja \
  -DCMAKE_INSTALL_PREFIX=/opt/drake-dependencies \
  /dependencies/src

ninja

ln -s /opt/drake-dependencies/bin/patchelf /usr/local/bin/patchelf
