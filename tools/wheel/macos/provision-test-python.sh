#!/usr/bin/env bash

# This scripts sets up the Python environment that will be used to test a Drake
# wheel on macOS. It can be run directly, but is normally run using the
# //tools/wheel:builder Bazel action.

set -eu -o pipefail

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <python-version>" >&2
    exit 1
fi

# Clean up from old tests.
rm -rf /opt/drake-wheel-test

# Prepare test environment.
mkdir /opt/drake-wheel-test

# NOTE: Xcode ships python3, make sure to use the one from brew.
$(brew --prefix python@$1)/bin/python$1 \
    -m venv /opt/drake-wheel-test/python$1
