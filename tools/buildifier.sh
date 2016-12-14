#!/bin/bash
#
# With arguments, fixes the given bazel file(s).  Without arguments, fixes
# every BUILD, .BUILD, and *.bzl file except third_party.
#
# TODO(jwnimmer-tri) Add WORKSPACE to the list of files to find and reformat,
# once buildifier rules stop murdering it.

set -e

workspace=$(dirname $(dirname "$0"))
if [ ! -f "$workspace"/WORKSPACE ]; then
  echo "$0: Cannot find WORKSPACE"
  exit 1
fi

echo "Refreshing buildifier binary ..."
bazel build --show_result=0 //tools:buildifier
buildifier="$workspace"/bazel-bin/external/com_github_bazelbuild_buildifier/buildifier/buildifier
if [ ! -x "$buildifier" ]; then
  echo "Failed to build buildifier at $buildifier"
  exit 1
fi

echo "Running buildifier ..."
if [ $# -gt 0 ]; then
  echo "$buildifier --mode=fix $@"
  "$buildifier" --mode=fix "$@"
else
  echo "Applying buildifier to everything! This may take a moment..."
  find "$workspace" \
       -name third_party -prune -o \
       \( -name BUILD -o \
          -name '*.BUILD' -o \
          -name '*.bzl' \) -print |
      xargs --verbose "$buildifier" -mode=fix
fi

echo "... done"
