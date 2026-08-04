#!/usr/bin/env bash

# This scripts sets up the Python environment that will be used to test a Drake
# wheel on macOS. It can be run directly, but is normally run using the
# //tools/wheel:builder Bazel action.

set -eu -o pipefail

readonly resource_root="$(
    cd "$(dirname "${BASH_SOURCE}")" && \
    realpath ..
)"

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <python-version>" >&2
    exit 1
fi

# Prepare test environment.
"$resource_root/image/provision-build.sh" "test-python$1" "test"

# NOTE: Xcode ships python3, make sure to use the one from brew.
$(brew --prefix python@$1)/bin/python$1 \
    -m venv /tmp/drake-wheel-test/python$1
