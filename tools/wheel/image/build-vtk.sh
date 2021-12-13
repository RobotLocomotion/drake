#!/bin/bash

set -e

VTK_VERSION=8.2.0

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
