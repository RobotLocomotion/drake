#!/bin/bash
#
# Run buildifier -mode=check on listed files; exit non-zero if any files need
# to be reformatted.

buildifier="$PWD/external/com_github_bazelbuild_buildtools/buildifier/buildifier"
tables="$PWD/tools/buildifier-tables.json"

out=$("$buildifier" -mode=check -add_tables="$tables" "$@")
[[ -z "$out" ]] && exit 0

for name in "$@"; do
  echo "error: $name:1: the buildifier-mandated formatting is incorrect" 1>&2
done
echo 'note: fix via tools/buildifier.sh '"$@" 1>&2
echo 'note: see http://drake.mit.edu/bazel.html#buildifier' 1>&2
echo "$out"
exit 1
