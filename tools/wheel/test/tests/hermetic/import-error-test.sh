#!/bin/bash

set -eu -o pipefail

cd "$(mktemp -d)"

root=/tmp/drake-wheel-test/python/lib/python*/site-packages/pydrake

# Remove libdrake.so so imports will fail.
# (Note: Because this test tampers with the test environment, it should only be
# run on Linux, where each test is run in a hermetic Docker container.)
rm $root/lib/libdrake.so

# Test that a failed import produces the expected error message.
python > out << EOF || true
import pydrake.common
EOF

trap 'cat out' ERR

grep -q 'Drake failed to load a required library' out
grep -q 'apt.*install' out
