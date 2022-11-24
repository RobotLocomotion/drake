#!/bin/bash

set -eu -o pipefail

mkdir -p /vtk
cd /vtk

# Defines `vtk_archive_name` used below, clones VTK into `${PWD}/src`.
. /image/common-definitions.sh
. /image/clone-vtk.sh

mkdir -p /vtk/build
cd /vtk/build

# Defines `vtk_cmake_args`.
drake_vtk_build_flavor=linux . /image/vtk-cmake-args.sh

set -x
cmake \
    -G Ninja \
    -Wno-dev \
    -DCMAKE_INSTALL_PREFIX:PATH=/opt/vtk \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    "${vtk_cmake_args[@]}" \
    /vtk/src
set +x

ninja install/strip

cd /opt/vtk

tar czf "${vtk_archive_name}" -- *
shasum --algorithm 256 "${vtk_archive_name}" | tee "${vtk_archive_name}.sha256"
