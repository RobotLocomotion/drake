#!/bin/bash
set -eux -o pipefail

# Copy necessary srcs to create a (set of) workspace(s) from an existing Bazel
# workspace. For testing Bazel workflows.

if [[ ! $(basename $(dirname ${PWD})) =~ .*\.runfiles ]]; then
    echo "Must be run from within Bazel"
    exit 1
fi

# Handle case when running with `bazel run`.
if [[ -z "${TEST_TMPDIR:-}" ]]; then
    tmp_base=/tmp/bazel_workspace_test
    mkdir -p "${tmp_base}"
    export TEST_TMPDIR=$(mktemp -d -p "${tmp_base}")
fi

# Declare the new workspace directory (do not use the root, as that will
# confuse Bazel with infinite symlinks).
workspace_dir="${TEST_TMPDIR}/workspace"
mkdir -p "${workspace_dir}"
# Copy all of runfiles, assuming that we have no directory symlinks.
srcs=$(find . -type f -o -type l)
for src in ${srcs}; do
    subdir=$(dirname "${src}")
    mkdir -p "${workspace_dir}/${subdir}"
    cp "${src}" "${workspace_dir}/${subdir}"
done

# Execute command.
cd "${workspace_dir}"
exec "$@"
