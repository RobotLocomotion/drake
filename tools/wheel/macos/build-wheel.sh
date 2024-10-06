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

if [[ $# -lt 2 ]]; then
    echo "Usage: $0 <drake-version> <python-version>" >&2
    exit 1
fi

readonly python_version="$2"
readonly python="python$python_version"

readonly python_prefix="$(brew --prefix python@$python_version)"
readonly python_executable="$python_prefix/bin/$python"

# -----------------------------------------------------------------------------
# Clean up from old builds.
# -----------------------------------------------------------------------------

rm -rf "/opt/drake-wheel-build/$python"
rm -rf "/opt/drake-dist/$python"

if [ -e "/opt/drake" ]; then
    echo "Unable to proceed: /opt/drake exists" \
         "(left over from a failed build?)" >&2
    exit 1
fi

# -----------------------------------------------------------------------------
# Build and "install" Drake.
# -----------------------------------------------------------------------------

readonly build_root="/opt/drake-wheel-build/$python/drake-build"

mkdir -p "$build_root"
cd "$build_root"

# Add wheel-specific bazel options.
cat > "$build_root/drake.bazelrc" << EOF
build --disk_cache=$HOME/.cache/drake-wheel-build/bazel/disk_cache
build --repository_cache=$HOME/.cache/drake-wheel-build/bazel/repository_cache
build --repo_env=DRAKE_OS=macos_wheel
build --repo_env=SNOPT_PATH=${SNOPT_PATH}
build --config=packaging
build --define=LCM_INSTALL_JAVA=OFF
# See tools/wheel/wheel_builder/macos.py for more on this env variable.
build --macos_minimum_os="${MACOSX_DEPLOYMENT_TARGET}"
EOF

# Install Drake.
cmake "$git_root" \
    -DDRAKE_VERSION_OVERRIDE="${DRAKE_VERSION}" \
    -DCMAKE_INSTALL_PREFIX="/opt/drake-dist/$python" \
    -DPython_EXECUTABLE="$python_executable"
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
# Install tools to build the wheel.
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Set up a Python virtual environment with the latest setuptools.
# -----------------------------------------------------------------------------

readonly pyvenv_root="/opt/drake-wheel-build/$python/python"

# NOTE: Xcode ships python3, make sure to use the one from brew.
"$python_executable" -m venv "$pyvenv_root"

# We also need pythonX.Y-config, which isn't created as of writing (see also
# https://github.com/pypa/virtualenv/issues/169). Don't fail if it already
# exists, though, e.g. if the bug has been fixed.
ln -s "$python_prefix/bin/$python-config" \
      "$pyvenv_root/bin/$python-config" || true # Allowed to already exist.

. "$pyvenv_root/bin/activate"

pip install --upgrade \
    delocate \
    setuptools \
    wheel

ln -s \
    "$build_root/bazel-bin/tools/wheel/strip_rpath" \
    "$pyvenv_root/bin/strip_rpath"

ln -s \
    "$build_root/bazel-bin/tools/wheel/change_lpath" \
    "$pyvenv_root/bin/change_lpath"

# -----------------------------------------------------------------------------
# Build the Drake wheel.
# -----------------------------------------------------------------------------

mkdir -p "/opt/drake-wheel-build/$python/wheel"

ln -s "/opt/drake-dist/$python" \
      "/opt/drake"

cp \
    "$resource_root/image/setup.py" \
    "/opt/drake-wheel-build/wheel/setup.py"

export DRAKE_VERSION="$1"

"$resource_root/image/build-wheel.sh"

rm "/opt/drake"
