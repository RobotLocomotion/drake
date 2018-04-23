#!/bin/bash
set -eux -o pipefail

# Creates and runs a Docker container which builds NumPy for Ubuntu 16.04.
# To be called on the host system.

cd $(dirname $0)

docker build -t numpy_builder .

mkdir -p build
cp ./build_direct.sh build
docker run --rm -v ${PWD}/build:/build numpy_builder \
    /build/build_direct.sh /build
