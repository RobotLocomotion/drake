#!/bin/bash
set -eux -o pipefail

# Ensure this is only run with `TEST_TMPDIR` present (called from
# `workspace_test.sh`).
[[ -n "${TEST_TMPDIR}" ]]

package_relpath="${1}"
shift

# Create mock drake/WORKSPACE file.
! test -f ./WORKSPACE
echo 'workspace(name = "drake")' > ./WORKSPACE
# Record directory.
drake_dir="${PWD}"

# Change to the workspace directory.
cd "${package_relpath}"
# Ensure path to Drake is corrected.
echo "def get_drake_path(_): return \"${drake_dir}\"" > ./drake_path.bzl
# Get rid of Bazel symlinks if they already exist.
rm bazel-* 2> /dev/null || :

# Run command.
exec "$@"
