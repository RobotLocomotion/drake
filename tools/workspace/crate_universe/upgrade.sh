#!/bin/bash

# upgrade.sh - Upgrades the lockfile for Drake's Rust crates.
#
# This program is only tested / supported on Ubuntu.

set -eu -o pipefail
cd $(dirname $(python3 -c 'import os; print(os.path.realpath("'"$0"'"))'))

# Ask @rules_rust to upgrade our lockfile and vendored BUILD rules.
bazel run :crate -- --repin=all
rm -f lock/details/crates.bzl

# Fix the upgrade advice comments.
old_tool="bazel run @//tools/workspace/crate_universe:crate"
new_tool="tools/workspace/crate_universe/upgrade.sh"
perl -pi -e "s#${old_tool}#${new_tool}#g;" lock/details/*

# Split off part of defs.bzl into a custom archives.bzl, so we have fine-grained
# control over exactly what Drake is downloading:
cp lock/details/defs.bzl lock/archives.bzl
perl -pi -e '
    BEGIN { undef $/; }
    s#def crate_repositories.*##s;
    ' lock/details/defs.bzl
perl -pi -e '
    BEGIN { undef $/; }
    s#.*def crate_repositories[^\n]*\n[^\n]*#ARCHIVES = [#s;
    s#maybe\(\n *http_archive,#dict(#g;
    s#\n\n#,\n#g;
    s#$#\n]#;
    s/^/# This file is automatically generated by upgrade.sh.\n/;
    ' lock/archives.bzl
