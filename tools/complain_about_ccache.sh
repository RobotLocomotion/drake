#!/bin/bash

# As a simple heuristic, we check if the PATH environment variable contains
# the string "ccache".  This will detect the common case, on both Linux and
# OS X, where the ccache compiler wrapper scripts live in a directory named
# ccache, and that directory appears in the path.  For instance, on Ubuntu
# the wrapper scripts live in /usr/lib/ccache.
#
# This heuristic will have a false negative if ccache wrapper scripts are
# present on the path in an unconventional directory.
#
# TODO(david-german-tri, soonho-tri): Can we detect ccache more robustly?
touch "$1"
echo $PATH | egrep "/ccache([:/]|$)" > /dev/null
if [ $? -eq 0 ]
then
  echo "" > "$1"
  echo "ccache is on your PATH, but Bazel cannot operate with ccache." >> "$1"
  echo "Please remove ccache from your PATH, then do the following to " >> "$1"
  echo "reassure Bazel that ccache is really gone: " >> "$1"
  echo "" >> "$1"
  # TODO(david-german-tri): Remove the kill step once we have updated to Bazel
  # 0.4.5 and added --action_env=PATH to our bazelrc. See bazelbuild/bazel#1143.
  echo "  kill \`bazel info server_pid\` && bazel clean" >> "$1"
  echo "" >> "$1"
  echo "Observed PATH: "$PATH >> "$1"
  echo "" >> "$1"
  cat "$1"
  exit 1
else
  echo "ccache is not on your path. Great!" > "$1"
  echo "Observed PATH: "$PATH >> "$1"
  echo "" >> "$1"
  exit 0
fi
