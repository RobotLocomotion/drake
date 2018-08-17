#!/bin/bash
# This should only be invoked via Bazel.
set -eux

capture_cc_env="$1"
source "$capture_cc_env"

"$BAZEL_CC" --version
