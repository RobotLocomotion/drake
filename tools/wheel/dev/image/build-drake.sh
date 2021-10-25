#!/bin/bash -e

cd /

git clone https://github.com/RobotLocomotion/drake

cd /drake

git apply < /image/pip-drake.patch

bazel run \
    --repo_env=DRAKE_OS=manylinux \
    --define NO_CLP=ON \
    --define NO_IPOPT=ON \
    --define NO_DREAL=ON \
    //:install -- /opt/drake
