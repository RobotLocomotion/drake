#!/usr/bin/env bash

# Internal script to build a macOS Drake wheel. This encapsulates most of the
# functionality that is handled by the Dockerfile when building Linux wheels.

set -eu -o pipefail

readonly resource_root="$(
    cd "$(dirname "${BASH_SOURCE}")" && \
    realpath ..
)"

readonly git_root="$(
    cd "$resource_root" && \
    realpath ./$(git rev-parse --show-cdup)
)"

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <drake-version>" >&2
    exit 1
fi

# -----------------------------------------------------------------------------
# Clean up from old builds.
# -----------------------------------------------------------------------------

rm -rf "/opt/drake-wheel-build/wheel"
rm -rf "/opt/drake"

# -----------------------------------------------------------------------------
# Build and "install" Drake.
# -----------------------------------------------------------------------------

readonly build_root="/opt/drake-wheel-build/drake-build"

mkdir -p "$build_root"
cd "$build_root"

# Add wheel-specific bazel options.
cat > "$build_root/drake.bazelrc" << EOF
build --repo_env=DRAKE_OS=macos_wheel
build --repo_env=SNOPT_PATH=git
build --config=packaging
# See tools/wheel/wheel_builder/macos.py for more on this env variable.
build --macos_minimum_os="${MACOSX_DEPLOYMENT_TARGET}"
EOF

# Install Drake.
cmake "$git_root" \
    -DCMAKE_INSTALL_PREFIX=/opt/drake
make install

# Build wheel tools.
cd "$build_root/drake_build_cwd"

bazel build //tools/wheel:strip_rpath
bazel build //tools/wheel:change_lpath

ln -s "$(bazel info bazel-bin)" "$build_root"/bazel-bin

# Ensure that the user (or build script) will be able to delete the build tree
# later.
find "$build_root" -type d -print0 | xargs -0 chmod u+w

# -----------------------------------------------------------------------------
# Set up a Python virtual environment with the latest setuptools.
# -----------------------------------------------------------------------------

rm -rf  "/opt/drake-wheel-build/python"

# NOTE: Xcode ships python3, make sure to use the one from brew.
$(brew --prefix python@3.11)/bin/python3.11 \
    -m venv "/opt/drake-wheel-build/python"

. "/opt/drake-wheel-build/python/bin/activate"

pip install --upgrade \
    delocate \
    setuptools \
    wheel

ln -s \
    "$build_root/bazel-bin/tools/wheel/strip_rpath" \
    "/opt/drake-wheel-build/python/bin/strip_rpath"

ln -s \
    "$build_root/bazel-bin/tools/wheel/change_lpath" \
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
