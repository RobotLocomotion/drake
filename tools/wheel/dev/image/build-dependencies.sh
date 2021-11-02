#!/bin/bash

set -e

mkdir -p /dependencies/build
cd /dependencies/build

cmake -G Ninja \
  -DCMAKE_INSTALL_PREFIX=/opt/drake-dependencies \
  /dependencies/src

ninja

ln -s /opt/drake-dependencies/lib/pkgconfig /usr/local/lib
ln -s /opt/drake-dependencies/share/pkgconfig /usr/local/share

ln -s /opt/drake-dependencies/bin/patchelf /usr/local/bin/patchelf
