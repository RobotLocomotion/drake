#!/bin/bash
set -e -u -x

# N.B. You must ensure that you have the proper build dependencies. See
# `Dockerfile` for Ubuntu files.

cd $(dirname $0)

cur_dir=${PWD}

mkdir -p build
cd build
${cur_dir}/build_direct.sh .
