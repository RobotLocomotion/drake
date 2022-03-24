#!/usr/bin/env bash

set -eu -o pipefail

# Use GNU 'cp' on macOS so we have a consistent CLI.
if [ "$(uname)" == "Darwin" ]; then
    cp()
    {
        gcp "$@"
    }
fi

# Helper function to change the RPATH of libraries. The first argument is the
# (origin-relative) new RPATH to be added. Remaining arguments are libraries to
# be modified. Note that existing RPATH(s) are removed.
chrpath()
{
    rpath=$1
    shift 1

    for lib in "$@"; do
        if [ "$(uname)" == "Linux" ]; then
            patchelf --remove-rpath "$lib"
            patchelf --set-rpath "\$ORIGIN/$rpath" "$lib"
        else
            strip_rpath "$lib"
            install_name_tool -add_rpath "@loader_path/$rpath" "$lib"
        fi
    done
}

###############################################################################

# TODO(mwoehlke-kitware) Most of this should move to Bazel.
mkdir -p ${DRAKE_WHEELBUILD_PREFIX}/wheel/drake
mkdir -p ${DRAKE_WHEELBUILD_PREFIX}/wheel/pydrake/lib
mkdir -p ${DRAKE_WHEELBUILD_PREFIX}/wheel/pydrake/share/drake
cd ${DRAKE_WHEELBUILD_PREFIX}/wheel

cp -r -t ${DRAKE_WHEELBUILD_PREFIX}/wheel/drake \
    /opt/drake/lib/python*/site-packages/drake/*

cp -r -t ${DRAKE_WHEELBUILD_PREFIX}/wheel/pydrake \
    /opt/drake/share/doc \
    /opt/drake/lib/python*/site-packages/pydrake/*

cp -r -t ${DRAKE_WHEELBUILD_PREFIX}/wheel/pydrake/lib \
    /opt/drake/lib/libdrake*.so

# NOTE: build-vtk.sh also puts licenses in /opt/drake-dependencies/licenses.
cp -r -t ${DRAKE_WHEELBUILD_PREFIX}/wheel/pydrake/doc \
    /opt/drake-dependencies/licenses/*

# TODO(mwoehlke-kitware) We need a different way of shipping non-arch files
# (examples, models).
cp -r -t ${DRAKE_WHEELBUILD_PREFIX}/wheel/pydrake/share/drake \
    /opt/drake/share/drake/.drake-find_resource-sentinel \
    /opt/drake/share/drake/package.xml \
    /opt/drake/share/drake/examples \
    /opt/drake/share/drake/geometry \
    /opt/drake/share/drake/manipulation \
    /opt/drake/share/drake/tutorials

if [ "$(uname)" == "Linux" ]; then
    mkdir -p ${DRAKE_WHEELBUILD_PREFIX}/wheel/pydrake/share/drake/setup
    cp -r -t ${DRAKE_WHEELBUILD_PREFIX}/wheel/pydrake/share/drake/setup \
        /opt/drake/share/drake/setup/deepnote
fi

# TODO(mwoehlke-kitware) We need to remove these to keep the wheel from being
# too large, but (per above), the whole of share/drake shouldn't be in the
# wheel.
rm -rf \
    ${DRAKE_WHEELBUILD_PREFIX}/wheel/pydrake/share/drake/manipulation/models/ycb/meshes/*.png \
    ${DRAKE_WHEELBUILD_PREFIX}/wheel/pydrake/share/drake/examples/atlas

if [ "$(uname)" == "Linux" ]; then
    export LD_LIBRARY_PATH=${DRAKE_WHEELBUILD_PREFIX}/wheel/pydrake/lib:/opt/drake-dependencies/lib
fi

chrpath lib pydrake/*.so
chrpath ../lib pydrake/*/*.so

if [ "$(uname)" == "Darwin" ]; then
    change_lpath '@loader_path/../../../' '@rpath/' pydrake/*.so
    change_lpath '@loader_path/../../../../' '@rpath/' pydrake/*/*.so
fi

python setup.py bdist_wheel

if [ "$(uname)" == "Linux" ]; then
    GLIBC_VERSION=$(ldd --version | sed -n '1{s/.* //;s/[.]/_/p}')

    auditwheel repair --plat manylinux_${GLIBC_VERSION}_x86_64 dist/drake*.whl
fi
