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

"$BAZEL_CC" --version
