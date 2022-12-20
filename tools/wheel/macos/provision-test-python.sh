#!/usr/bin/env bash

# This scripts sets up the Python environment that will be used to test a Drake
# wheel on macOS. It can be run directly, but is normally run using the
# //tools/wheel:builder Bazel action.

set -eu -o pipefail

# Clean up from old tests.
rm -rf /opt/drake-wheel-test

# Prepare test environment.
# NOTE: Xcode ships python3, make sure to use the one from brew.
$(brew --prefix python@3.10)/bin/python3.10 \
    -m venv /opt/drake-wheel-test/python
