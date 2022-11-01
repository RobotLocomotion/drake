#!/bin/bash

set -eu -o pipefail

# Can be a branch, tag, or commit (e.g., 'v9.1.0').
readonly VTK_VERSION="d706250a1422ae1e7ece0fa09a510186769a5fec"

mkdir -p /vtk
cd /vtk

git clone https://gitlab.kitware.com/vtk/vtk.git src
git -C src checkout "${VTK_VERSION}"

mkdir -p /vtk/build
cd /vtk/build

mapfile -t VTK_CMAKE_ARGS < <(sed -e '/^#/d' -e 's/^/-D/' < /vtk/vtk-args)

codename="$(lsb_release -cs)"
if [[ "${codename}" == "focal" ]]; then
    cxx_std="17"
else  # "${code_name}" := "jammy"
    cxx_std="20"
fi

cmake \
    -G Ninja \
    -Wno-dev \
    -DCMAKE_INSTALL_PREFIX:PATH=/opt/vtk \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    "-DCMAKE_CXX_STANDARD:STRING=${cxx_std}" \
    "${VTK_CMAKE_ARGS[@]}" \
    /vtk/src

ninja install/strip
