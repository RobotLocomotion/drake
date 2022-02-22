#!/bin/bash

set -eu -o pipefail

chrpath()
{
    rpath=$1
    shift 1

    for lib in "$@"; do
        patchelf --remove-rpath "$lib"
        patchelf --set-rpath "$rpath" "$lib"
    done
}

# TODO(mwoehlke-kitware) Most of this should move to Bazel.
mkdir -p /wheel/drake
mkdir -p /wheel/pydrake/lib
mkdir -p /wheel/pydrake/share/drake
cd /wheel

cp -r -t /wheel/drake \
    /opt/drake/lib/python*/site-packages/drake/*

cp -r -t /wheel/pydrake \
    /opt/drake/share/doc \
    /opt/drake/lib/python*/site-packages/pydrake/*

cp -r -t /wheel/pydrake/lib \
    /opt/drake/lib/libdrake*.so

# NOTE: build-vtk.sh also puts licenses in /opt/drake-dependencies/licenses.
cp -r -t /wheel/pydrake/doc \
    /opt/drake-dependencies/licenses/*

# TODO(mwoehlke-kitware) We need a different way of shipping non-arch files
# (examples, models).
cp -r -t /wheel/pydrake/share/drake \
    /opt/drake/share/drake/.drake-find_resource-sentinel \
    /opt/drake/share/drake/package.xml \
    /opt/drake/share/drake/examples \
    /opt/drake/share/drake/geometry \
    /opt/drake/share/drake/manipulation

mkdir -p /wheel/pydrake/share/drake/setup
cp -r -t /wheel/pydrake/share/drake/setup \
    /opt/drake/share/drake/setup/deepnote

# TODO(mwoehlke-kitware) We need to remove these to keep the wheel from being
# too large, but (per above), the whole of share/drake shouldn't be in the
# wheel.
rm /wheel/pydrake/share/drake/manipulation/models/ycb/meshes/*.png
rm -r /wheel/pydrake/share/drake/examples/atlas

export LD_LIBRARY_PATH=/wheel/pydrake/lib:/opt/drake-dependencies/lib

chrpath '$ORIGIN/lib' pydrake/*.so
chrpath '$ORIGIN/../lib' pydrake/*/*.so

python setup.py bdist_wheel

GLIBC_VERSION=$(ldd --version | sed -n '1{s/.* //;s/[.]/_/p}')

auditwheel repair --plat manylinux_${GLIBC_VERSION}_x86_64 dist/drake*.whl
