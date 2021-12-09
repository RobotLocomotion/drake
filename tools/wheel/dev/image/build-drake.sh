#!/bin/bash

set -e

cd /drake

git apply < /image/pip-drake.patch

export SNOPT_PATH=git

# TODO(svenevs) Bionic continues to ICE with gcc 7 and Eigen code, reducing
# --local_{cpu,ram}_resources is only solution.  Remove with Bionic support.
bazel run \
    --local_cpu_resources=HOST_CPUS*0.5 \
    --local_ram_resources=HOST_RAM*0.5 \
    --disk_cache=/var/cache/bazel/disk_cache \
    --repository_cache=/var/cache/bazel/repository_cache \
    --repo_env=DRAKE_OS=manylinux \
    --define NO_CLP=ON \
    --define NO_IPOPT=ON \
    --define NO_DREAL=ON \
    --define WITH_SNOPT=ON \
    //:install -- /opt/drake
