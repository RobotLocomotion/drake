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
build --repo_env=DRAKE_WHEEL=1
build --repo_env=SNOPT_PATH=${SNOPT_PATH}
build --config=packaging
build --define=LCM_INSTALL_JAVA=OFF
# See tools/wheel/wheel_builder/macos.py for more on this env variable.
build --macos_minimum_os="${MACOSX_DEPLOYMENT_TARGET}"
# TODO(jwnimmer-tri) Ideally this would be automatically inferred.
common --@drake//tools/install/libdrake:spdlog_dynamic=False
EOF

# Install Drake.
cmake "$git_root" \
    -DDRAKE_VERSION_OVERRIDE="${DRAKE_VERSION}" \
    -DCMAKE_INSTALL_PREFIX="/opt/drake-dist/$python" \
    -DPython_EXECUTABLE="$python_executable"
make install

# Build wheel tools.
cd "$build_root/drake_build_cwd"

bazel build @drake//tools/wheel:strip_rpath
bazel build @drake//tools/wheel:change_lpath

ln -s "$(bazel info bazel-bin)" "$build_root"/bazel-bin

# Ensure that the user (or build script) will be able to delete the build tree
# later.
find "$build_root" -type d -print0 | xargs -0 chmod u+w

# -----------------------------------------------------------------------------
# "Install" additional tools to build the wheel.
# -----------------------------------------------------------------------------

ln -nsf "$git_root" "/opt/drake-wheel-build/drake"

readonly venv_drake="/opt/drake-wheel-build/drake/venv"

ln -s \
    "$build_root/bazel-bin/external/drake+/tools/wheel/strip_rpath" \
    "$venv_drake/bin/strip_rpath"

ln -s \
    "$build_root/bazel-bin/external/drake+/tools/wheel/change_lpath" \
    "$venv_drake/bin/change_lpath"

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
