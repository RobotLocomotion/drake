#!/usr/bin/env bash

# This script tests a wheel on macOS. It can be run directly, but is normally
# run using the //tools/wheel:builder Bazel action.

set -eu -o pipefail

readonly pip_root="$(cd "$(dirname "${BASH_SOURCE}")" && realpath ..)"

if [ $# -lt 1 ]; then
    echo "Usage: $0 <wheel>" >&2
    exit 1
fi

readonly wheel="$(realpath "$1")"

# -----------------------------------------------------------------------------
# Clean up from old tests and prepare test environment.
# -----------------------------------------------------------------------------

rm -rf "${DRAKE_WHEELBUILD_PREFIX}/test/python"

python3 -m venv "${DRAKE_WHEELBUILD_PREFIX}/test/python"

cd "${DRAKE_WHEELBUILD_PREFIX}/test/python"

# -----------------------------------------------------------------------------
# Install the wheel and run the tests.
# -----------------------------------------------------------------------------

"$pip_root/test/test-wheel.sh" "$wheel"

# -----------------------------------------------------------------------------
# Remove the test environment.
# -----------------------------------------------------------------------------

cd /

rm -rf "${DRAKE_WHEELBUILD_PREFIX}/test/python"
