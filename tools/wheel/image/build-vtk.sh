#!/usr/bin/env bash

# Internal script to build VTK for use in a Drake wheel.
#
# NOTE: If you need to update this file, please refer to
#       tools/workspace/vtk/README for important details.

set -eu -o pipefail

readonly VTK_DIR=/opt/drake-wheel-build/vtk
readonly VTK_VERSION=9.1.0

mkdir -p ${VTK_DIR}
cd ${VTK_DIR}

git clone \
    --branch v${VTK_VERSION} --depth 1 \
    https://gitlab.kitware.com/vtk/vtk.git src

mkdir -p ${VTK_DIR}/build
cd ${VTK_DIR}/build

mapfile -t VTK_CMAKE_ARGS < \
    <(sed -e '/^#/d' -e 's/^/-D/' < ${VTK_DIR}/vtk-args)

cmake "${VTK_CMAKE_ARGS[@]}" -GNinja -Wno-dev ${VTK_DIR}/src

ninja install/strip

# Place the license files from VTK where build-wheel.sh will copy from.
# See also: vtk-args, CMAKE_INSTALL_PREFIX/CMAKE_INSTALL_LICENSEDIR.
mkdir -p /opt/drake-dependencies/licenses
cp -r /opt/vtk/share/doc/vtk-9.1 /opt/drake-dependencies/licenses
