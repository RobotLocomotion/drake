#!/bin/bash
set -eux -o pipefail

# @file
# Removes `bazel-*` symlinks (which mess up Bazel's own package scanning)
# underneath `//tools/external_data/test/...`.
# `bazel test ...` does not like having workspace sym-links, as it tries to
# analyze them (tripping itself up) rather than ignore them.

cd $(dirname "${0}")
find . -type l -name 'bazel-*' | xargs rm || :
