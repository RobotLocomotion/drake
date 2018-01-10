#!/bin/bash
set -e -u

cd $(dirname ${0})

# `bazel test ...` does not like having workspace sym-links, as it tries to
# analyze them (tripping itself up) rather than ignore them.
find . -type l -name 'bazel-*' | xargs rm || :
