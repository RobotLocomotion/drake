#!/bin/bash

echo $PATH | grep ccache 2> /dev/null
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
  echo "Observed path: "$PATH
  echo ""
  exit 1
else
  exit 0
fi
