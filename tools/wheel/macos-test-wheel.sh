#!/usr/bin/env bash

# This script tests a wheel on macOS. It can be run directly, but is normally
# run using the accompanying build-wheels script.
#
# Beware that this requires write permission to /opt and will nuke /opt/python.

set -eu -o pipefail

readonly pip_root="$(cd "$(dirname "${BASH_SOURCE}")" && realpath .)"

if [ $# -lt 1 ]; then
    echo "Usage: $0 <wheel>" >&2
    exit 1
fi

readonly wheel="$(realpath "$1")"

# -----------------------------------------------------------------------------
# Clean up from old tests and prepare test environment.
# -----------------------------------------------------------------------------

rm -rf /opt/python

python3 -m venv /opt/python

cd /opt/python

# -----------------------------------------------------------------------------
# Install the wheel and run the tests.
# -----------------------------------------------------------------------------

"$pip_root/test/test-wheel.sh" "$wheel"

# -----------------------------------------------------------------------------
# Remove the test environment.
# -----------------------------------------------------------------------------

cd /

rm -rf /opt/python
