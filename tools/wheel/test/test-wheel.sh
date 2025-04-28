#!/bin/bash

# This shell script runs a test on a Drake wheel. It must be run inside of a
# container (Ubuntu) or environment (macOS) which has been properly
# provisioned, e.g. by the accompanying Dockerfile (Ubuntu) or by
# macos/provision-wheel-python.sh (macOS), and which already has the wheel
# installed. The path to the test script must be given as an argument to the
# script.
#
# In general, it is not recommended to attempt to run this script directly;
# use //tools/wheel:builder instead.

set -eu -o pipefail

if [[ -z "$1" ]]; then
    echo "Usage: $0 <test> <wheel>" >&2
    exit 1
fi

cd "$(dirname "${BASH_SOURCE}")"

. /tmp/drake-wheel-test/python/bin/activate

exec "$@"
