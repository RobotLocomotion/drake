#!/bin/bash

set -eu -o pipefail

cd "$(mktemp -d)"

root=/opt/drake-wheel-test/python/lib/python*/site-packages/pydrake
doc=$root/INSTALLATION

# Ensure that INSTALLATION exists.
find $root -name INSTALLATION
[ -f $doc ]

# Test showInstallInstructions() and ensure it's output is as expected.
python > out << EOF
import pydrake
pydrake.showInstallInstructions()
EOF

diff -u $doc out
