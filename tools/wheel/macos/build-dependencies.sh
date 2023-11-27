#!/usr/bin/env bash

# Internal script to build dependencies for a macOS Drake wheel. This is a
# wrapper over the generic script that performs additional environment
# preparation.

set -eu -o pipefail

readonly resource_root="$(
    cd "$(dirname "${BASH_SOURCE}")" && \
    realpath ..
)"

# gfortran hard-codes the path to the SDK with which it was built, which may
# not match the SDK actually on the machine. This can result in the error
# "ld: library not found for -lm", and can be fixed/overridden by setting
# SDKROOT to the appropriate path.
export SDKROOT="$(xcrun --show-sdk-path)"

# Ensure nothing is left from previous builds.
rm -rf /opt/drake-dependencies

rm -rf "/opt/drake-wheel-build/dependencies"
mkdir -p "/opt/drake-wheel-build/dependencies"

# Copy the main script into the correct location and run it.
cp -R \
    "$resource_root/image/dependencies" \
    "/opt/drake-wheel-build/dependencies/src"

"$resource_root/image/build-dependencies.sh"
