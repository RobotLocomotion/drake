#!/bin/bash

set -e

cd /drake

git apply < /image/pip-drake.patch

export SNOPT_PATH=git

bazel run \
    --disk_cache=/var/cache/bazel/disk_cache \
    --repository_cache=/var/cache/bazel/repository_cache \
    --repo_env=DRAKE_OS=manylinux \
    --define NO_DRAKE_VISUALIZER=ON \
    --define NO_CLP=ON \
    --define NO_IPOPT=ON \
    --define NO_DREAL=ON \
    --define WITH_SNOPT=ON \
    //:install -- /opt/drake
