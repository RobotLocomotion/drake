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
echo $PATH | egrep "/ccache([:/]|$)" > /dev/null
if [ $? -eq 0 ]
then
  echo ""
  echo "ccache appears on your PATH, but Bazel cannot interoperate with ccache."
  echo "Please remove ccache from your PATH, then do the following to reassure "
  echo "Bazel that ccache is really gone: "
  echo ""
  # TODO(david-german-tri): Remove the kill step once we have updated to a Bazel
  # release that includes the fix for bazelbuild/bazel#1143.
  echo "  kill \`bazel info server_pid\` && bazel clean"
  echo ""
  echo "Observed PATH: "$PATH
  echo ""
  exit 1
else
  exit 0
fi
