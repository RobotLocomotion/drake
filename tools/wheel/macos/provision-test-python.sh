#!/usr/bin/env bash

# This scripts sets up the Python environment that will be used to test a Drake
# wheel on macOS. It can be run directly, but is normally run using the
# //tools/wheel:builder Bazel action.

set -eu -o pipefail

# Clean up from old tests.
rm -rf /opt/drake-wheel-test

# Prepare test environment.
python3 -m venv /opt/drake-wheel-test/python
