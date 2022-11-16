#!/usr/bin/env bash

# Internal script to build VTK for use in a Drake wheel.
#
# NOTE: If you need to update this file, please refer to
#       tools/workspace/vtk/README for important details.

set -eu -o pipefail

readonly VTK_DIR=/opt/drake-wheel-build/vtk

mkdir -p ${VTK_DIR}
cd ${VTK_DIR}

# This file is executed both (a) in Docker for linux wheel, and (b) natively
# for macOS wheel.  The path to the common files needs to be detected.
if command -v lsb_release >&/dev/null; then
    common_definitions_sh_path="/image/common-definitions.sh"
    clone_vtk_sh_path="/image/clone-vtk.sh"
    vtk_cmake_args_sh_path="/image/vtk-cmake-args.sh"
    flavor="linux_wheel"
else
    this_file_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
    common_definitions_sh_path="${this_file_dir}/common-definitions.sh"
    clone_vtk_sh_path="${this_file_dir}/clone-vtk.sh"
    vtk_cmake_args_sh_path="${this_file_dir}/vtk-cmake-args.sh"
    flavor="mac_wheel"
fi

# Defines `codename`, clones VTK into `${PWD}/src`.
. "${common_definitions_sh_path}"
. "${clone_vtk_sh_path}"

# Defines `vtk_cmake_args`.
drake_vtk_build_flavor="${flavor}" . "${vtk_cmake_args_sh_path}"

mkdir -p ${VTK_DIR}/build
cd ${VTK_DIR}/build

set -x
cmake \
    -G Ninja \
    -Wno-dev \
    -DCMAKE_PREFIX_PATH:PATH=/opt/drake-dependencies \
    -DCMAKE_INSTALL_PREFIX:PATH=/opt/vtk \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    "${vtk_cmake_args[@]}" \
    "${VTK_DIR}/src"
set +x

ninja install/strip

# Place the license files from VTK where build-wheel.sh will copy from.
# See also: vtk-args, CMAKE_INSTALL_PREFIX/CMAKE_INSTALL_LICENSEDIR.
mkdir -p /opt/drake-dependencies/licenses
cp -r /opt/vtk/share/doc/vtk-9.2 /opt/drake-dependencies/licenses
