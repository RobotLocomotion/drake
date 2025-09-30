#!/bin/bash

# upgrade.sh - Upgrades the lockfile for Drake's Rust crates.
#
# This program is only tested / supported on Ubuntu.

set -eu -o pipefail
cd $(dirname $(python3 -c 'import os; print(os.path.realpath("'"$0"'"))'))

# Ask @rules_rust to upgrade our lockfile and vendored BUILD rules; this bumps
# everything under the `lock` subdirectory except for `repo_names.bzl` which is
# custom to Drake, and handled separately below.
bazel run :crate -- --repin=all

# Fix the upgrade advice comments in the generated BUILD files.
old_tool="bazel run @@//tools/workspace/crate_universe:crate"
new_tool="tools/workspace/crate_universe/upgrade.sh"
perl -pi -e "s#${old_tool}#${new_tool}#g;" lock/details/*

# Regenerate the Drake-specific metadata (the list of repository names).
echo "REPO_NAMES = [" > lock/repo_names.bzl
(cd lock/details && ls BUILD.*-*.bazel) | sort |
    sed -e 's#^BUILD\.#    "crate__#; s#\.bazel$#",#g;' \
    >> lock/repo_names.bzl
echo "]" >> lock/repo_names.bzl

# Stage the changes.
git add lock
