#!/bin/bash

# Internal script to build Python.
# Docker (Linux) only.

set -eu -o pipefail

readonly VERSION=$1
readonly PREFIX=$2
readonly SHA=$3

apt-get -y update
apt-get -y install --no-install-recommends \
    build-essential \
    libc6-dev \
    libssl-dev

readonly ARCHIVE=Python-$VERSION.tar.xz
readonly URL=https://www.python.org/ftp/python/$VERSION/$ARCHIVE
readonly SRC_DIR=/opt/drake-wheel-build/python

mkdir -p $SRC_DIR
cd $SRC_DIR

wget $URL
echo "$SHA  $ARCHIVE" | sha256sum -c

tar --strip-components=1 -xf $ARCHIVE
rm $ARCHIVE

./configure --prefix=$PREFIX
make -j install
