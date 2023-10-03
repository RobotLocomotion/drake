#!/bin/bash

# Internal script to build a Drake from which a wheel will be created.
# Docker (Linux) only.

set -eu -o pipefail

cd /opt/drake-wheel-build/drake

# TODO(jwnimmer-tri) Switch to using the CMake for wheel builds so that we can
# use the canonical `-DPython_EXECUTABLE` instead.
export _DRAKE_WHEEL_BUILD_PYTHON_INTERPRETER_PATH=/usr/local/bin/python

# Store downloads in the build cache to speed up rebuilds.
export BAZELISK_HOME=/var/cache/bazel/bazelisk

export SNOPT_PATH=git

third_party/com_github_bazelbuild_bazelisk/bazelisk.py run \
    --disk_cache=/var/cache/bazel/disk_cache \
    --repository_cache=/var/cache/bazel/repository_cache \
    --repo_env=DRAKE_OS=manylinux \
    --config=omp \
    --define=WITH_MOSEK=ON \
    --define=WITH_SNOPT=ON \
    //:install -- /opt/drake
