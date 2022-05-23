#!/usr/bin/env bash

# Internal script to build a wheel from a Drake installation.

set -eu -o pipefail

if [ "$(uname)" == "Darwin" ]; then
    HOMEBREW="$(brew config | \grep -E '^HOMEBREW_PREFIX' | cut -c18-)"

    # Use GNU 'cp' on macOS so we have a consistent CLI.
    cp()
    {
        gcp "$@"
    }
fi

# Helper function to change the RPATH of libraries. The first argument is the
# (origin-relative) new RPATH to be added. Remaining arguments are libraries to
# be modified. Note that existing RPATH(s) are removed (except for homebrew
# RPATHs on macOS).
chrpath()
{
    rpath=$1
    shift 1

    for lib in "$@"; do
        if [ "$(uname)" == "Linux" ]; then
            patchelf --remove-rpath "$lib"
            patchelf --set-rpath "\$ORIGIN/$rpath" "$lib"
        else
            strip_rpath --exclude="$HOMEBREW" "$lib"
            install_name_tool -add_rpath "@loader_path/$rpath" "$lib"
        fi
    done
}

###############################################################################

# TODO(mwoehlke-kitware) Most of this should move to Bazel.
mkdir -p ${DRAKE_WHEELBUILD_PREFIX:-}/wheel/drake
mkdir -p ${DRAKE_WHEELBUILD_PREFIX:-}/wheel/pydrake/lib
mkdir -p ${DRAKE_WHEELBUILD_PREFIX:-}/wheel/pydrake/share/drake
cd ${DRAKE_WHEELBUILD_PREFIX:-}/wheel

cp -r -t ${DRAKE_WHEELBUILD_PREFIX:-}/wheel/drake \
    /opt/drake/lib/python*/site-packages/drake/*

cp -r -t ${DRAKE_WHEELBUILD_PREFIX:-}/wheel/pydrake \
    /opt/drake/share/doc \
    /opt/drake/lib/python*/site-packages/pydrake/*

cp -r -t ${DRAKE_WHEELBUILD_PREFIX:-}/wheel/pydrake/lib \
    /opt/drake/lib/libdrake*.so

# NOTE: build-vtk.sh also puts licenses in /opt/drake-dependencies/licenses.
cp -r -t ${DRAKE_WHEELBUILD_PREFIX:-}/wheel/pydrake/doc \
    /opt/drake-dependencies/licenses/*

# MOSEK is "sort of" third party, but is procured as part of Drake's build and
# ends up in /opt/drake.
if [ "$(uname)" == "Darwin" ]; then
    # On macOS, it is explicitly referenced by @loader_path, and thus must be
    # copied to the same place as libdrake.so.
    cp -r -t ${DRAKE_WHEELBUILD_PREFIX:-}/wheel/pydrake/lib \
        /opt/drake/lib/libmosek*.dylib \
        /opt/drake/lib/libcilkrts*.dylib
else
    # On Linux, it needs to be copied somewhere where auditwheel can find it.
    cp -r -t /opt/drake-dependencies/lib \
        /opt/drake/lib/libmosek*.so* \
        /opt/drake/lib/libcilkrts*.so*
fi

# TODO(mwoehlke-kitware) We need a different way of shipping non-arch files
# (examples, models).
cp -r -t ${DRAKE_WHEELBUILD_PREFIX:-}/wheel/pydrake/share/drake \
    /opt/drake/share/drake/.drake-find_resource-sentinel \
    /opt/drake/share/drake/package.xml \
    /opt/drake/share/drake/examples \
    /opt/drake/share/drake/geometry \
    /opt/drake/share/drake/manipulation \
    /opt/drake/share/drake/tutorials

if [ "$(uname)" == "Linux" ]; then
    mkdir -p ${DRAKE_WHEELBUILD_PREFIX:-}/wheel/pydrake/share/drake/setup
    cp -r -t ${DRAKE_WHEELBUILD_PREFIX:-}/wheel/pydrake/share/drake/setup \
        /opt/drake/share/drake/setup/deepnote
fi

# TODO(mwoehlke-kitware) We need to remove these to keep the wheel from being
# too large, but (per above), the whole of share/drake shouldn't be in the
# wheel.
rm -rf \
    ${DRAKE_WHEELBUILD_PREFIX:-}/wheel/pydrake/share/drake/manipulation/models/ycb/meshes/*.png \
    ${DRAKE_WHEELBUILD_PREFIX:-}/wheel/pydrake/share/drake/examples/atlas

if [ "$(uname)" == "Linux" ]; then
    export LD_LIBRARY_PATH=${DRAKE_WHEELBUILD_PREFIX:-}/wheel/pydrake/lib:/opt/drake-dependencies/lib
fi

chrpath lib pydrake/*.so
chrpath ../lib pydrake/*/*.so

if [ "$(uname)" == "Darwin" ]; then
    change_lpath '@loader_path/../../../' '@rpath/' pydrake/*.so
    change_lpath '@loader_path/../../../../' '@rpath/' pydrake/*/*.so
fi

python setup.py bdist_wheel

if [ "$(uname)" == "Darwin" ]; then
    delocate-wheel -w wheelhouse -v dist/drake*.whl
else
    GLIBC_VERSION=$(ldd --version | sed -n '1{s/.* //;s/[.]/_/p}')

    auditwheel repair --plat manylinux_${GLIBC_VERSION}_x86_64 dist/drake*.whl
fi
