#!/usr/bin/env bash

# This script builds a wheel on macOS. It can be run directly, but using the
# //tools/wheel:builder Bazel action adds functionality. Running this script
# directly also requires an already-provisioned host.
#
# Beware that this requires write permission to /opt and will nuke various
# things therein. (Shouldn't affect ARM Homebrew, though.)

set -eu -o pipefail

readonly resource_root="$(
    cd "$(dirname "${BASH_SOURCE}")" && \
    realpath ..
)"

readonly git_root="$(
    cd "$resource_root" && \
    realpath ./$(git rev-parse --show-cdup)
)"

build_deps=1
if [[ "$1" == "--no-deps" ]]; then
    build_deps=
    shift 1
fi

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <drake-version>" >&2
    exit 1
fi

# -----------------------------------------------------------------------------
# Clean up from old builds and prepare build environment.
# -----------------------------------------------------------------------------

rm -rf "/opt/drake-wheel-build/wheel"
rm -rf "/opt/drake"

mkdir -p ~/.ssh
chmod 700 ~/.ssh

# Merge image/known_hosts into the user's known_hosts. Primarily, this ensures
# that GitHub's SSH fingerprint is known.
while read host proto fingerprint; do
    grep -qE '^github.com' ~/.ssh/known_hosts && continue
    echo "$host $proto $fingerprint" >> ~/.ssh/known_hosts
done < "$resource_root/image/known_hosts"

chmod 600 ~/.ssh/known_hosts

# gfortran hard-codes the path to the SDK with which it was built, which may
# not match the SDK actually on the machine. This can result in the error
# "ld: library not found for -lm", and can be fixed/overridden by setting
# SDKROOT to the appropriate path.
export SDKROOT="$(xcrun --show-sdk-path)"

# -----------------------------------------------------------------------------
# Build Drake's dependencies.
# -----------------------------------------------------------------------------

if [[ -n "$build_deps" ]]; then
    rm -rf /opt/drake-dependencies

    rm -rf "/opt/drake-wheel-build/dependencies"
    mkdir -p "/opt/drake-wheel-build/dependencies"

    cp -R \
        "$resource_root/image/dependencies" \
        "/opt/drake-wheel-build/dependencies/src"

    "$resource_root/image/build-dependencies.sh"

    rm -rf /opt/vtk

    rm -rf "/opt/drake-wheel-build/vtk"
    mkdir -p "/opt/drake-wheel-build/vtk"

    "$resource_root/image/build-vtk.sh"
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

rm -rf  "/opt/drake-wheel-build/python"

python3 -m venv "/opt/drake-wheel-build/python"

. "/opt/drake-wheel-build/python/bin/activate"

pip install --upgrade \
    delocate \
    setuptools \
    wheel

ln -s \
    "$git_root/bazel-bin/tools/wheel/strip_rpath" \
    "/opt/drake-wheel-build/python/bin/strip_rpath"

ln -s \
    "$git_root/bazel-bin/tools/wheel/change_lpath" \
    "/opt/drake-wheel-build/python/bin/change_lpath"

# -----------------------------------------------------------------------------
# Build the Drake wheel.
# -----------------------------------------------------------------------------

mkdir -p "/opt/drake-wheel-build/wheel"

cp \
    "$resource_root/image/setup.py" \
    "/opt/drake-wheel-build/wheel/setup.py"

export DRAKE_VERSION="$1"

"$resource_root/image/build-wheel.sh"
