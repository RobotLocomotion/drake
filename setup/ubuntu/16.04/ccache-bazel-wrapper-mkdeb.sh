#!/bin/bash

# This creates a deb release of ccache-bazel-wrapper.sh.
# If run with "--install", it will also install the resulting deb.

set -eu

cd $(dirname "$0")

# Sanity check.
[ -e ccache-bazel-wrapper.sh ]
[ ! -e draketmp ]
rm -f ccache-bazel-wrapper.tgz

# Setup directories.
mkdir -p draketmp/usr/lib/ccache/
mkdir -p draketmp/usr/lib/ccache-bazel-wrapper/
mkdir -p draketmp/usr/share/doc/ccache-bazel-wrapper/

# Add files.
cp -a ./ccache-bazel-wrapper.sh draketmp/usr/lib/ccache-bazel-wrapper/bazel
cp -a ../../../LICENSE.TXT draketmp/usr/share/doc/ccache-bazel-wrapper/copyright
(cd draketmp/usr/lib/ccache/ && ln -s ../ccache-bazel-wrapper/bazel)

# Archive.
(cd draketmp && tar cfz ../ccache-bazel-wrapper.tgz ./)
rm -rf draketmp
fakeroot alien \
  --description="Compatability shim to turn off ccache when invoking bazel" \
  --version="0-0" \
  --fixperms \
  --to-deb ccache-bazel-wrapper.tgz
rm -f ccache-bazel-wrapper.tgz

# Final product.
[ -e ccache-bazel-wrapper_0-0-2_all.deb ]

# Optional install.
if [ x"$1" == x"--install" ]; then
    sudo dpkg -i ccache-bazel-wrapper_0-0-2_all.deb
    rm -f ccache-bazel-wrapper_0-0-2_all.deb
else
    set +x
    echo "... success!"
    echo
    echo "To install, run:"
    echo "sudo dpkg -i "$(pwd)"/ccache-bazel-wrapper_0-0-2_all.deb"
fi
