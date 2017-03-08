#!/bin/bash
### Builds libdrake.so, and packages it with the required headers.

bazel build //drake:libdrake.so //drake:libdrake_headers

# Copy off all the package artifacts into a temporary directory.
workspace=`bazel info workspace`
cd $workspace
tmpdir=`mktemp -d`
mkdir -p $tmpdir/lib
mkdir -p $tmpdir/include/scratch
cp bazel-bin/drake/libdrake.so $tmpdir/lib
cp bazel-bin/drake/libdrake_headers.tar.gz $tmpdir/include/scratch

# Un-tar the headers. The -P flag and subsequent cleanup is necessary because
# Bazel bakes a .. into the paths of external headers.
cd $tmpdir/include/scratch
tar -xzfP libdrake_headers.tar.gz
mv drake ..
rm libdrake_headers.tar.gz
cd ..
rmdir scratch

# TODO(david-german-tri): Normalize this directory tree in accordance with
# Drake inclusion conventions, so that ``-I path/to/package/include` suffices.
