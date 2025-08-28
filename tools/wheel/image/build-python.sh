#!/bin/bash

# Internal script to build Python.
# Docker (Linux) only.

set -eu -o pipefail

readonly VERSION=$1
readonly PREFIX=$2
readonly EXPECTED_SHA=$3

readonly ARCHIVE=Python-$VERSION.tar.xz
readonly URL=https://www.python.org/ftp/python/$VERSION/$ARCHIVE
readonly SRC_DIR=/tmp/drake-wheel-build/python-src

mkdir -p $SRC_DIR
cd $SRC_DIR

wget --no-verbose $URL

readonly ACTUAL_SHA=$(sha256sum $ARCHIVE | cut -d' ' -f1)
echo "    Actual SHA: $ACTUAL_SHA"
echo "  Expected SHA: $EXPECTED_SHA"
test $ACTUAL_SHA == $EXPECTED_SHA

tar --strip-components=1 -xf $ARCHIVE
rm $ARCHIVE

./configure --prefix=$PREFIX
make -j install
