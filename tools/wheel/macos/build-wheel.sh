#!/usr/bin/env bash

# This script builds a wheel on macOS. It requires an already-provisioned host.
#
# Beware that this requires write permission to /opt and will nuke various
# things therein. (Shouldn't affect ARM Homebrew, though.)

set -eu -o pipefail

readonly pip_root="$(cd "$(dirname "${BASH_SOURCE}")" && realpath ..)"
readonly git_root="$(cd "$pip_root" && realpath ./$(git rev-parse --show-cdup))"

build_deps=1
if [ "$1" == "--no-deps" ]; then
    build_deps=
    shift 1
fi

if [ $# -lt 1 ]; then
    echo "Usage: $0 <drake-version>" >&2
    exit 1
fi

# -----------------------------------------------------------------------------
# Clean up from old builds and prepare build environment.
# -----------------------------------------------------------------------------

rm -rf "${DRAKE_WHEELBUILD_PREFIX}/wheel"

if [ -d /opt/drake ]; then
    # We need to ensure there are no remnants of a prior build, but CI creates
    # a wheel output directory in /opt/drake that we don't have permission to
    # remove. Thus, we have to get a little creative...
    find /opt/drake \
        \! -path /opt/drake/wheelhouse \
        \! -path /opt/drake \
        -delete
fi

mkdir -p ~/.ssh
chmod 700 ~/.ssh

while read host proto fingerprint; do
    grep -qE '^github.com' ~/.ssh/known_hosts && continue
    echo "$host $proto $fingerprint" >> ~/.ssh/known_hosts
done < "$pip_root/image/known_hosts"

chmod 600 ~/.ssh/known_hosts

# -----------------------------------------------------------------------------
# Build Drake's dependencies.
# -----------------------------------------------------------------------------

if [ -n "$build_deps" ]; then
    rm -rf /opt/drake-dependencies

    rm -rf "${DRAKE_WHEELBUILD_PREFIX}/dependencies"
    mkdir -p "${DRAKE_WHEELBUILD_PREFIX}/dependencies"

    cp -R \
        "$pip_root/image/dependencies" \
        "${DRAKE_WHEELBUILD_PREFIX}/dependencies/src"

    "$pip_root/image/build-dependencies.sh"

    rm -rf /opt/vtk

    rm -rf "${DRAKE_WHEELBUILD_PREFIX}/vtk"
    mkdir -p "${DRAKE_WHEELBUILD_PREFIX}/vtk"

    cp \
        "$pip_root/image/vtk-args" \
        "${DRAKE_WHEELBUILD_PREFIX}/vtk/vtk-args"

    "$pip_root/image/build-vtk.sh"
fi

# -----------------------------------------------------------------------------
# Build and "install" Drake.
# -----------------------------------------------------------------------------

cd "$git_root"

export SNOPT_PATH=git

declare -a bazel_args=(
    --repo_env=DRAKE_OS=macos_wheel
    --define NO_DRAKE_VISUALIZER=ON
    --define NO_DREAL=ON
    --define WITH_MOSEK=ON
    --define WITH_SNOPT=ON
)

bazel build "${bazel_args[@]}" //tools/wheel:strip_rpath
bazel build "${bazel_args[@]}" //tools/wheel:change_lpath

bazel run "${bazel_args[@]}" //:install -- /opt/drake

# -----------------------------------------------------------------------------
# Set up a Python virtual environment with the latest setuptools.
# -----------------------------------------------------------------------------

rm -rf  "${DRAKE_WHEELBUILD_PREFIX}/python"

python3 -m venv "${DRAKE_WHEELBUILD_PREFIX}/python"

. "${DRAKE_WHEELBUILD_PREFIX}/python/bin/activate"

pip install --upgrade \
    delocate \
    setuptools \
    wheel

ln -s \
    "$git_root/bazel-bin/tools/wheel/strip_rpath" \
    "${DRAKE_WHEELBUILD_PREFIX}/python/bin/strip_rpath"

ln -s \
    "$git_root/bazel-bin/tools/wheel/change_lpath" \
    "${DRAKE_WHEELBUILD_PREFIX}/python/bin/change_lpath"

# -----------------------------------------------------------------------------
# Build the Drake wheel.
# -----------------------------------------------------------------------------

mkdir -p "${DRAKE_WHEELBUILD_PREFIX}/wheel"

cp \
    "$pip_root/image/setup.py" \
    "${DRAKE_WHEELBUILD_PREFIX}/wheel/setup.py"

export DRAKE_VERSION="$1"

"$pip_root/image/build-wheel.sh"
