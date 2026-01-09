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
# Build and "install" Drake.
# -----------------------------------------------------------------------------

# Create a unique space for the build. Because there may be multiple such
# spaces if the user needs to inspect them (i.e. using --keep-build), prefix
# the space with the Python version to aid in identifying the spaces.
"$resource_root/image/provision-build.sh" "build-$python"

readonly build_root="/tmp/drake-wheel-build/drake-build"

mkdir -p "$build_root"
cd "$build_root"

# Add wheel-specific bazel options.
# N.B. When you change anything here, also fix wheel/image/build-drake.sh.
cat > "$build_root/drake.bazelrc" << EOF
build --disk_cache=$HOME/.cache/drake-wheel-build/bazel/disk_cache
build --repository_cache=$HOME/.cache/drake-wheel-build/bazel/repository_cache
build --repo_env=DRAKE_WHEEL=1
build --repo_env=SNOPT_PATH=${SNOPT_PATH}
build --config=packaging
# See tools/wheel/wheel_builder/macos.py for more on this env variable.
build --macos_minimum_os="${MACOSX_DEPLOYMENT_TARGET}"
EOF

# MOSEK's published wheels declare an upper bound on their supported Python
# version, which is currently Python < 3.15. When that changes to a larger
# version number, we should bump this up to match, and also grep tools/wheel
# for other mentions of MOSEK version bounds and fix those as well.
PYTHON_MINOR=$($python_executable -c "import sys; print(sys.version_info.minor)")
if [[ ${PYTHON_MINOR} -ge 15 ]]; then
    cat >> "$build_root/drake.bazelrc" << EOF
build --@drake//tools/flags:with_mosek=False
EOF
fi

# Install Drake.
# N.B. When you change anything here, also fix wheel/image/build-drake.sh.
cmake "$git_root" \
    -DWITH_USER_EIGEN=OFF \
    -DWITH_USER_FMT=OFF \
    -DWITH_USER_SPDLOG=OFF \
    -DWITH_USER_BLAS=OFF \
    -DWITH_USER_LAPACK=OFF \
    -DWITH_USER_ZLIB=OFF \
    -DDRAKE_INSTALL_JAVA=OFF \
    -DDRAKE_VERSION_OVERRIDE="${DRAKE_VERSION}" \
    -DCMAKE_INSTALL_PREFIX="/tmp/drake-wheel-build/drake-dist" \
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

ln -nsf "$git_root" "/tmp/drake-wheel-build/drake-src"

readonly venv_drake="/tmp/drake-wheel-build/drake-src/venv"

ln -s \
    "$build_root/bazel-bin/external/drake+/tools/wheel/strip_rpath" \
    "$venv_drake/bin/strip_rpath"

ln -s \
    "$build_root/bazel-bin/external/drake+/tools/wheel/change_lpath" \
    "$venv_drake/bin/change_lpath"

# -----------------------------------------------------------------------------
# Build the Drake wheel.
# -----------------------------------------------------------------------------

mkdir -p "/tmp/drake-wheel-build/drake-wheel"

cp \
    "$resource_root/image/setup.py" \
    "/tmp/drake-wheel-build/drake-wheel/setup.py"

export DRAKE_VERSION="$1"

"$resource_root/image/build-wheel.sh"
