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

cmake "${VTK_CMAKE_ARGS[@]}" -GNinja -Wno-dev /vtk/src

ninja install/strip
