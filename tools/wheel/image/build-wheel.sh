#!/usr/bin/env bash

# Internal script to build a wheel from a Drake installation.

set -eu -o pipefail

if [[ "$(uname)" == "Darwin" ]]; then
    HOMEBREW="$(brew --prefix)"

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
        if [[ "$(uname)" == "Linux" ]]; then
            patchelf --remove-rpath "$lib"
            patchelf --set-rpath "\$ORIGIN/$rpath" "$lib"
        else
            strip_rpath \
                --exclude="$HOMEBREW" \
                "$lib"
            install_name_tool -add_rpath "@loader_path/$rpath" "$lib"
        fi
    done
}

# Helper function to copy the imported copyright text into the wheel's
# documentation.
copy_license()
{
    package_name=$1
    mkdir -p ${WHEEL_DIR}/pydrake/doc/${package_name}
    cp -t ${WHEEL_DIR}/pydrake/doc/${package_name}/ \
        /tmp/drake-wheel-build/drake-wheel-licenses/${package_name}/copyright
}

###############################################################################

# Activate Drake's virtual environment, which provides some of the tools that
# we need to build the wheels.
. /tmp/drake-wheel-build/drake-src/venv/bin/activate

readonly WHEEL_DIR=/tmp/drake-wheel-build/drake-wheel
readonly WHEEL_SHARE_DIR=${WHEEL_DIR}/pydrake/share

# TODO(mwoehlke-kitware) Most of this should move to Bazel.
mkdir -p ${WHEEL_DIR}/drake
mkdir -p ${WHEEL_DIR}/pydrake/lib
mkdir -p ${WHEEL_DIR}/pydrake/share/drake
cd ${WHEEL_DIR}

cp -r -t ${WHEEL_DIR}/drake \
    /tmp/drake-wheel-build/drake-dist/lib/python*/site-packages/drake/*

cp -r -t ${WHEEL_DIR}/pydrake \
    /tmp/drake-wheel-build/drake-dist/share/doc \
    /tmp/drake-wheel-build/drake-dist/lib/python*/site-packages/pydrake/*

cp -r -t ${WHEEL_DIR}/pydrake/lib \
    /tmp/drake-wheel-build/drake-dist/lib/libdrake*.so

# MOSEK's published wheels declare an upper bound on their supported Python
# version, which is currently Python < 3.15. When that changes to a larger
# version number, we should bump this up to match, and also grep tools/wheel
# for other mentions of MOSEK version bounds and fix those as well.
PYTHON_MINOR=$(python -c "import sys; print(sys.version_info.minor)")
MOSEK_ENABLED=1
[ ${PYTHON_MINOR} -ge 15 ] && MOSEK_ENABLED=

if [[ "$(uname)" == "Darwin" && -n "${MOSEK_ENABLED}" ]]; then
    # MOSEK is "sort of" third party, but is procured as part of Drake's build
    # and ends up in /tmp/drake-wheel-build/drake-dist/. It should end up in
    # the same place as libdrake.so.
    cp -r -t ${WHEEL_DIR}/pydrake/lib \
        /tmp/drake-wheel-build/drake-dist/lib/libmosek* \
        /tmp/drake-wheel-build/drake-dist/lib/libtbb*
fi

if [[ "$(uname)" == "Linux" ]]; then
  cp -r -t ${WHEEL_DIR}/pydrake \
      /tmp/drake-wheel-build/drake-wheel-content/*
fi

# Copy the license files from third party dependencies we vendor.
if [[ "$(uname)" == "Linux" ]]; then
    # The drake/tools/wheel/test/tests/libs-test.py must be kept in sync with
    # this list. To maintain that correspondence, the _ALLOWED_LIBS entry seen
    # in that test program is added as comment to the end of each line below.
    copy_license gcc  # libgfortran, libgomp, libquadmath
fi

cp -r -t ${WHEEL_SHARE_DIR}/drake \
    /tmp/drake-wheel-build/drake-dist/share/drake/.drake-find_resource-sentinel \
    /tmp/drake-wheel-build/drake-dist/share/drake/package.xml \
    /tmp/drake-wheel-build/drake-dist/share/drake/examples \
    /tmp/drake-wheel-build/drake-dist/share/drake/geometry \
    /tmp/drake-wheel-build/drake-dist/share/drake/multibody \
    /tmp/drake-wheel-build/drake-dist/share/drake/tutorials

if [[ "$(uname)" == "Linux" ]]; then
    export LD_LIBRARY_PATH=${WHEEL_DIR}/pydrake/lib
fi

chrpath lib pydrake/*.so
chrpath ../lib pydrake/*/*.so

if [[ "$(uname)" == "Darwin" ]]; then
    change_lpath \
        --old='@loader_path/../../../' \
        --new='@rpath/' \
        pydrake/*.so
    change_lpath \
        --old='@loader_path/../../../../' \
        --new='@rpath/' \
        pydrake/*/*.so
fi

python -m build --wheel

if [[ "$(uname)" == "Darwin" ]]; then
    delocate-wheel -w wheelhouse -v dist/drake*.whl

    # Remove libmosek from wheels.
    if [[ -n "${MOSEK_ENABLED}" ]]; then
        for w in wheelhouse/drake*.whl; do
            mkdir fixup-wheel
            wheel unpack --dest fixup-wheel "$w"

            rm fixup-wheel/drake-*/pydrake/lib/libmosek*
            rm fixup-wheel/drake-*/pydrake/lib/libtbb*
            rm fixup-wheel/drake-*/pydrake/doc/mosek/mosek-eula.pdf
            rm fixup-wheel/drake-*/pydrake/doc/mosek/LICENSE.third_party

            change_lpath \
                --old='@loader_path/libtbb' \
                --old='@loader_path/libmosek' \
                --new='@loader_path/../../mosek/libtbb' \
                --new='@loader_path/../../mosek/libmosek' \
                fixup-wheel/drake-*/pydrake/lib/*.so
            change_lpath \
                --old='@loader_path/lib/libtbb' \
                --old='@loader_path/lib/libmosek' \
                --new='@loader_path/../mosek/libtbb' \
                --new='@loader_path/../mosek/libmosek' \
                fixup-wheel/drake-*/pydrake/*.so
            change_lpath \
                --old='@loader_path/../lib/libtbb' \
                --old='@loader_path/../lib/libmosek' \
                --new='@loader_path/../../mosek/libtbb' \
                --new='@loader_path/../../mosek/libmosek' \
                fixup-wheel/drake-*/pydrake/*/*.so

            rm "$w"
            wheel pack --dest wheelhouse fixup-wheel/drake-*/
            rm -rf fixup-wheel
        done
    fi
else
    GLIBC_VERSION=$(ldd --version | sed -n '1{s/.* //;s/[.]/_/p}')

    auditwheel repair --plat manylinux_${GLIBC_VERSION}_x86_64 dist/drake*.whl
fi
