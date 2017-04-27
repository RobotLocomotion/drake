#!/bin/bash
### Builds libdrake.so, and packages it with the required headers.
### @param1 the name of the output file, which should end in .tar.gz.
### After installing the package, OS X users may want to adjust the .so file's
### id to match its installed location, using install_name_tool -id.

# If any command in the script exits non-zero, stop.
set -e

[ -n "$1" ] || (echo "error: Please specify an output file." && false)

mydir=$(dirname "$0")
outfile="$(cd $(dirname "$1") && pwd)/$(basename "$1")"
touch "$outfile"
cd "$mydir"

# Build Drake.
workspace=$(bazel info workspace)
cd "$workspace"
bazel build //drake:libdrake.so //drake:libdrake_headers //drake:external_licenses

# Copy off all the package artifacts into a temporary directory.
# TODO(jamiesnape): Add drake-config.cmake and drake-targets.cmake.
tmpdir=$(mktemp -d)
mkdir -p "$tmpdir/lib/cmake/drake"
mkdir -p "$tmpdir/include/external/scratch"
cp bazel-bin/drake/libdrake.so "$tmpdir/lib"
cp bazel-bin/drake/libdrake_headers.tar.gz "$tmpdir/include/external/scratch"
cp bazel-bin/drake/external_licenses.tar.gz "$tmpdir/include/external"
cp tools/drake-config.cmake "$tmpdir/lib/cmake/drake"
chmod -R 755 "$tmpdir"

# Un-tar the headers. The -P flag and scratch directory are necessary because
# Bazel bakes a .. into the paths of external headers.
cd "$tmpdir/include/external/scratch"
tar -xPzf libdrake_headers.tar.gz

# Headers from Drake proper are now inside include/external/scratch, while
# headers from externals are in external/scratch.  Move the Drake headers up
# to include/drake.
mv drake "$tmpdir/include"

# Clean up the header tarball, and the scratch directory.
rm libdrake_headers.tar.gz
cd .. # include/external
rmdir scratch

# Make a list of the externals.
extlist=$(mktemp)
ls -d1 */ | sed "s#/\$##g" > "$extlist"
# Make a list of the directories for which we have license files.
liclist=$(mktemp)
tar tf external_licenses.tar.gz | sed "s#\./##g" | xargs dirname | sort | uniq | grep -v "\." > "$liclist"
# Confirm those two lists are identical.
diff "$extlist" "$liclist" || (echo "error: There should be a one-to-many relationship between externals and license files." && false)

# Un-tar the license notices.
tar -xzf external_licenses.tar.gz
rm external_licenses.tar.gz

# Copy the license for Drake itself.
cp "$workspace/LICENSE.TXT" $tmpdir

# Make the package tarball.
cd "$tmpdir"
tar -czf "$outfile" ./*
