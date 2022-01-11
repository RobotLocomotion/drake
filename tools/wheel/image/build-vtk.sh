#!/bin/bash

# NOTE: If you need to update this file, please refer to
#       tools/workspace/vtk/README for important details.

set -eu -o pipefail

readonly VTK_VERSION=9.1.0

mkdir -p /vtk
cd /vtk

git clone \
    --branch v${VTK_VERSION} --depth 1 \
    https://gitlab.kitware.com/vtk/vtk.git src

mkdir -p /vtk/build
cd /vtk/build

mapfile -t VTK_CMAKE_ARGS < <(sed -e '/^#/d' -e 's/^/-D/' < /vtk/vtk-args)

cmake "${VTK_CMAKE_ARGS[@]}" -GNinja -Wno-dev /vtk/src

ninja install/strip
