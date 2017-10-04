#!/bin/bash
#
# TODO(jwnimmer-tri) This is a compatibility shim; remove this file on or
# around 2017-11-01.

set -e

echo "WARNING: buildifier.sh is deprecated; use bazel-bin/tools/lint/buildifier" 1>&2

workspace=$(cd $(dirname $0) && bazel info workspace)
if [[ ! -f "$workspace/WORKSPACE" ]]; then
  echo "$0: Cannot find WORKSPACE"
  exit 1
fi

exec "$workspace"/bazel-bin/tools/lint/buildifier "$@"
