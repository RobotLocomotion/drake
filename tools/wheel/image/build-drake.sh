#!/bin/bash

# Internal script to build a Drake from which a wheel will be created.
# Docker (Linux) only.

set -eu -o pipefail

cd /opt/drake-wheel-build/drake

git apply < /image/pip-drake.patch

export SNOPT_PATH=git

bazel run \
    --disk_cache=/var/cache/bazel/disk_cache \
    --repository_cache=/var/cache/bazel/repository_cache \
    --repo_env=DRAKE_OS=manylinux \
    --config=omp \
    --define=WITH_MOSEK=ON \
    --define=WITH_SNOPT=ON \
    //:install -- /opt/drake
