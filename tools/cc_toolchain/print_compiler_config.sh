#!/bin/bash
# This should only be invoked via Bazel.
set -eu

capture_compiler_env="$1"
source "$capture_compiler_env"

[ -n "$CC" ]
[ -n "$CXX" ]

echo "CC=$(type -P "$CC")"
echo "CXX=$(type -P "$CXX")"
