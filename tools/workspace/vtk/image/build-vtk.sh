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
    -DCMAKE_C_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wno-deprecated-declarations' \
    -DCMAKE_CXX_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wno-deprecated-declarations' \
    -DCMAKE_EXE_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DCMAKE_MODULE_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DCMAKE_SHARED_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    "${VTK_CMAKE_ARGS[@]}" \
    /vtk/src

ninja install/strip
