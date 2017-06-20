#!/bin/bash
#
# Run buildifier -mode=check on listed files; exit non-zero if any files need
# to be reformatted.

buildifier="$PWD/external/com_github_bazelbuild_buildtools/buildifier/buildifier"

out=$("$buildifier" -mode=check "$@")
[[ -z "$out" ]] && exit 0

echo "$out"
exit 1
