#!/bin/bash
set -e -u

# Ensure this is only run with `TEST_TMPDIR` present (called from
# `workspace_test.sh`).
[[ -n "${TEST_TMPDIR}" ]]

pkg_reldir=${1}
shift

# Create mock drake/WORKSPACE file.
! test -f ./WORKSPACE
echo 'workspace(name = "drake")' > ./WORKSPACE
# Record directory.
drake_dir=${PWD}

# Change to the workspace directory.
cd ${pkg_reldir}
# Ensure path to Drake is corrected.
sed -i "s#path = .*,#path = \"${drake_dir}\",#g" ./WORKSPACE
# Get rid of Bazel symlinks if they already exist.
rm bazel-* 2> /dev/null || :

# Run command.
eval "$@"
