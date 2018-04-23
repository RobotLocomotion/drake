#!/bin/bash
set -eux -o pipefail

# Builds NumPy for Mac. To be called on the host system.

cd $(dirname $0)

cur_dir=${PWD}

mkdir -p build
cd build
${cur_dir}/build_direct.sh .
