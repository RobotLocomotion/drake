#!/bin/bash
# This should only be invoked via Bazel.
set -eux

# TODO(jamiesnape): Determine this information from Bazel.
if [[ "$(uname -s)" == Darwin ]]; then
  export DEVELOPER_DIR="$(xcode-select --print-path)"
  export SDKROOT="$(xcrun --show-sdk-path)"
fi

capture_cc_env="$1"
source "$capture_cc_env"

[[ ! -z "$BAZEL_CC" ]]
BAZEL_CC=$(python3 -c 'import os; print(os.path.realpath("'"$BAZEL_CC"'"))')
"$BAZEL_CC" --version
