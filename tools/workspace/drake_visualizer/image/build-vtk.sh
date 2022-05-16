#!/bin/bash

set -eu -o pipefail

readonly VTK_VERSION=8.2.0

mkdir -p /vtk
cd /vtk

git clone \
    --branch v${VTK_VERSION} --depth 1 \
    https://gitlab.kitware.com/vtk/vtk.git src

cd /vtk/src
git apply /vtk/patches/*.patch

mkdir -p /vtk/build
cd /vtk/build

mapfile -t VTK_CMAKE_ARGS < <(sed -e '/^#/d' -e 's/^/-D/' < /vtk/vtk-args)

cmake "${VTK_CMAKE_ARGS[@]}" -GNinja -Wno-dev /vtk/src

ninja install/strip

find /opt/vtk -name __pycache__ -type d -print0 | xargs -0 rm -rf

sed --in-place \
    's|VTK_RUNTIME_DIRS "/opt/vtk/bin"|VTK_RUNTIME_DIRS "${VTK_INSTALL_PREFIX}/bin"|g' \
    /opt/vtk/lib/cmake/vtk-*/VTKConfig.cmake
sed --in-place \
    's|VTK_RUNTIME_DIRS "/opt/vtk/lib"|VTK_RUNTIME_DIRS "${VTK_INSTALL_PREFIX}/lib"|g' \
    /opt/vtk/lib/cmake/vtk-*/VTKConfig.cmake
sed --in-place \
    's|/opt/vtk|${_IMPORT_PREFIX}|g' \
    /opt/vtk/lib/cmake/vtk-*/VTKTargets.cmake
sed --in-place \
    's|VTK_PYTHONPATH "/opt/vtk/lib/python3"|VTK_PYTHONPATH "${VTK_INSTALL_PREFIX}/lib/python3"|g' \
    /opt/vtk/lib/cmake/vtk-*/Modules/vtkPython.cmake
