#!/bin/bash

set -eu -o pipefail

readonly PYTHON_VERSINFO=($(python3 -c 'import sys; print(*sys.version_info)'))
readonly python=python${PYTHON_VERSINFO[0]}.${PYTHON_VERSINFO[1]}

# Note that we remove several installed components that are either broken,
# unused by Drake, or duplicated by Drake. Exactly which are broken vs. unused
# vs. duplicated has been lost in time, may be wrong now.
#
# TODO(jamiesnape): Maybe we should bundle some of the duplicate components
# anyway.
rm -rf \
    /opt/director/bin/directorPython \
    /opt/director/include \
    /opt/director/lib/*.a \
    /opt/director/lib/bot2-core \
    /opt/director/lib/cmake \
    /opt/director/lib/pkgconfig \
    /opt/director/lib/python3 \
    /opt/director/lib/robotlocomotion-lcmtypes

find /opt/director -name __pycache__ -type d -print0 | xargs -0 rm -rf

# Copy in VTK libraries and Python bindings.
cp  /opt/vtk/lib/*.so.1 \
    /opt/director/lib

cp  /opt/vtk/lib/${python}/site-packages/vtk.py \
    /opt/director/lib/${python}/site-packages/

cp -r \
    /opt/vtk/lib/${python}/site-packages/vtkmodules \
    /opt/director/lib/${python}/site-packages/

# We have to manually call "strip" since there is no "install/strip" target
# when building drake-visualizer and its dependencies as external projects.
strip \
    /opt/director/bin/drake-visualizer \
    /opt/director/lib/*.so \
    /opt/director/lib/*.so.1 \
    /opt/director/lib/${python}/site-packages/director/*.so \
    /opt/director/lib/python3.*/site-packages/director/thirdparty/*.so

# Fix the rpath entries so that components can find their corresponding VTK-8
# counterparts.  This includes ensuring that VTK-8 libraries can find one
# another ($ORIGIN), as well as python site-packages/{module}/*.so can find
# the original VTK-8 libraries ($ORIGIN/../../..).
find /opt/director/lib -type f -name "libvtk*-8.2.so.1" \
    -mindepth 1 -maxdepth 1 \
    -printf 'patchelf on %p\n' \
    -exec patchelf --set-rpath '$ORIGIN' {} \;
readonly site_packages="/opt/director/lib/${python}/site-packages"
find "${site_packages}/vtkmodules" -type f -name "*.so" \
    -mindepth 1 -maxdepth 1 \
    -printf 'patchelf on %p\n' \
    -exec patchelf --set-rpath '$ORIGIN/../../..' {} \;
find "${site_packages}/director" -type f -name "vtk*.so" \
    -mindepth 1 -maxdepth 1 \
    -printf 'patchelf on %p\n' \
    -exec patchelf --set-rpath '$ORIGIN/../../..' {} \;
# Additionally, the rpath entries for the director components get set to
# /opt/director/lib:/opt/vtk/lib:/opt/drake/lib which can create conflicts
# when the machine where drake is being installed has these directories.
# Overwrite them to refer to the components within this tarball.
patchelf --set-rpath '$ORIGIN' /opt/director/lib/libddApp.so
patchelf --set-rpath '$ORIGIN/../lib' /opt/director/bin/drake-visualizer
# Lastly, _transformations.so has an rpath leading to /opt as well.  This
# library depends on system components only, but we remove its rpath for
# posterity and to avoid potential collisions.
patchelf --remove-rpath \
    "${site_packages}/director/thirdparty/_transformations.so"

# Get various version numbers.
readonly dv_tag=dv-$(cd /director/src; git describe)
readonly py_tag=python-$(echo ${PYTHON_VERSINFO[@]:0:3} | tr ' ' '.')
readonly qt_tag=qt-$(moc --version | awk '{print $2}')
readonly vtk_tag=vtk-8.2.0
readonly platform=$(lsb_release --codename --short)-$(uname --processor)

# Create archive named:
#   dv-<version>
#     -python-<python version>
#     -qt-<qt version>
#     -vtk-<vtk version>
#     -<distribution codename>
#     -<processor architecture>
#     -<build_number>.tar.gz
#
# The <build_number> was introduced to distinguish VTK-8/VTK-9 director split.
readonly archive=${dv_tag}-${py_tag}-${qt_tag}-${vtk_tag}-${platform}-5.tar.gz

cd /opt/director

tar czf $archive -- *
shasum --algorithm 256 $archive | tee $archive.sha256
