#!/bin/bash
# This should only be invoked via Bazel.
set -eu

capture_compiler_env="$1"
source "$capture_compiler_env"

[ -n "$BAZEL_CC" ]
[ -n "$BAZEL_CXX" ]

echo "CC=$(type -P "$BAZEL_CC")"
echo "CXX=$(type -P "$BAZEL_CXX")"
