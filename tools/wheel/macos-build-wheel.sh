#!/usr/bin/env bash

set -eu -o pipefail

readonly pip_root="$(cd "$(dirname "${BASH_SOURCE}")" && realpath .)"
readonly git_root="$(cd "$pip_root" && realpath ./$(git rev-parse --show-cdup))"

if [ $# -lt 1 ]; then
    echo "Usage: $0 <drake-version>" >&2
    exit 1
fi

# -----------------------------------------------------------------------------
# Clean up from old builds and prepare build environment.
# -----------------------------------------------------------------------------

rm -rf \
    "${DRAKE_WHEELBUILD_PREFIX}/wheel" \
    "${DRAKE_WHEELBUILD_PREFIX}/vtk" \
    "${DRAKE_WHEELBUILD_PREFIX}/dependencies" \
    /opt/drake-dependencies \
    /opt/drake

brew bundle --file="$pip_root/image/packages-macos" --no-lock

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

mkdir -p "${DRAKE_WHEELBUILD_PREFIX}/dependencies"

cp -R \
    "$pip_root/image/dependencies" \
    "${DRAKE_WHEELBUILD_PREFIX}/dependencies/src"

"$pip_root/image/build-dependencies.sh"


mkdir -p "${DRAKE_WHEELBUILD_PREFIX}/vtk"

cp \
    "$pip_root/image/vtk-args" \
    "${DRAKE_WHEELBUILD_PREFIX}/vtk/vtk-args"

"$pip_root/image/build-vtk.sh"

# -----------------------------------------------------------------------------
# Build and "install" Drake.
# -----------------------------------------------------------------------------

cd "$git_root"

export SNOPT_PATH=git

bazel run \
    --repo_env=DRAKE_OS=macos_wheel \
    --define NO_DRAKE_VISUALIZER=ON \
    --define NO_DREAL=ON \
    --define WITH_SNOPT=ON \
    //:install -- /opt/drake

# -----------------------------------------------------------------------------
# Set up a Python virtual environment with the latest setuptools.
# -----------------------------------------------------------------------------

rm -rf  /opt/python

python3 -m venv /opt/python

. /opt/python/bin/activate

pip install --upgrade pip
pip install --upgrade setuptools
pip install --upgrade wheel

cp \
    "$pip_root/image/strip_rpath.py" \
    /opt/python/bin/strip_rpath

cp \
    "$pip_root/image/change_lpath.py" \
    /opt/python/bin/change_lpath

# -----------------------------------------------------------------------------
# Build the Drake wheel.
# -----------------------------------------------------------------------------

mkdir -p "${DRAKE_WHEELBUILD_PREFIX}/wheel"

cp \
    "$pip_root/image/setup.py" \
    "${DRAKE_WHEELBUILD_PREFIX}/wheel/setup.py"

export DRAKE_VERSION="$1"

"$pip_root/image/build-wheel.sh"
