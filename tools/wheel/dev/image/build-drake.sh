#!/bin/bash

set -e

cd /drake

if [ $(ls | wc -l) -eq 1 ] && [ -f *.tar.gz ]; then
    tar --strip-components=1 -xf *.tar.gz
fi

git apply < /image/pip-drake.patch

export SNOPT_PATH=git

bazel run \
    --repo_env=DRAKE_OS=manylinux \
    --define NO_CLP=ON \
    --define NO_IPOPT=ON \
    --define NO_DREAL=ON \
    --define WITH_SNOPT=ON \
    //:install -- /opt/drake
