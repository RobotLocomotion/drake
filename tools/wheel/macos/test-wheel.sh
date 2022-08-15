#!/usr/bin/env bash

# This script tests a wheel on macOS. It can be run directly, but is normally
# run using the //tools/wheel:builder Bazel action.

set -eu -o pipefail

readonly resource_root="$(cd "$(dirname "${BASH_SOURCE}")" && realpath ..)"

if [ $# -lt 1 ]; then
    echo "Usage: $0 <wheel>" >&2
    exit 1
fi

readonly wheel="$(realpath "$1")"

# -----------------------------------------------------------------------------
# Clean up from old tests and prepare test environment.
# -----------------------------------------------------------------------------

rm -rf /opt/drake-wheel-test

python3 -m venv /opt/drake-wheel-test/python

cd /opt/drake-wheel-test/python

# -----------------------------------------------------------------------------
# Install the wheel and run the tests.
# -----------------------------------------------------------------------------

"$resource_root/test/test-wheel.sh" "$wheel"

# -----------------------------------------------------------------------------
# Remove the test environment.
# -----------------------------------------------------------------------------

cd /  # ...so we don't remove $PWD.

rm -rf /opt/drake-wheel-test
