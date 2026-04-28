#!/bin/bash
# This should only be invoked via Bazel.
set -eu

capture_compiler_env="$1"
source "$capture_compiler_env"

[ -n "$CC" ]

echo "CC=$(type -P "$CC")"
