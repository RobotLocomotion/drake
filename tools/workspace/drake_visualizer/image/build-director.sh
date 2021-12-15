#!/bin/bash

set -eu -o pipefail

readonly DIRECTOR_SHA=4c3e570a6797ff840c74067c742455daaa113d93

mkdir -p /director
cd /director

git clone \
    --branch master --single-branch \
   https://github.com/RobotLocomotion/director.git  src

cd /director/src
git checkout ${DIRECTOR_SHA}
git apply /director/patches/*.patch

mkdir -p /director/build
cd /director/build

mapfile -t DIRECTOR_CMAKE_ARGS \
    < <(sed -e '/^#/d' -e 's/^/-D/' < /director/director-args)

cmake \
    "${DIRECTOR_CMAKE_ARGS[@]}" \
    -GNinja -Wno-deprecated -Wno-dev \
    /director/src/distro/superbuild

export PYTHONWARNINGS=ignore::SyntaxWarning

ninja
