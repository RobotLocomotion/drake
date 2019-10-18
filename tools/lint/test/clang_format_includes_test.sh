#!/bin/bash

# A basic _acceptance_ test for clang-format-includes.
# More specific _unit_ testing is in formatter_test.py.
# This test should only be run via Bazel, never directly on the command line.

set -ex

# Fail if not run under Bazel.
[[ -n "${TEST_TMPDIR}" ]]

# Dump some debugging output.
find .

# Prep.
dut=./tools/lint/clang-format-includes
[ -x "$dut" ]
mkdir -p drake/dummy

# Set up a test input that is mis-formatted and the correct formatting.
cat > drake/dummy/foo.cc <<EOF
#include "drake/common/drake_assert.h"
#include "drake/dummy/bar.h"
#include "drake/dummy/foo.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <algorithm>
#include <poll.h>
#include <sys/wait.h>
#include <vector>
EOF
cp -a drake/dummy/foo.cc{,.orig}
cat > drake/dummy/foo.cc-expected <<EOF
#include "drake/dummy/foo.h"

#include <poll.h>
#include <sys/wait.h>

#include <algorithm>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/dummy/bar.h"
EOF

# The foo.cc needs reformatting, but check-only leaves it unchanged.
(set +e; "$dut" --check-only drake/dummy/foo.cc; [ $? == 1 ])
diff -U 999 drake/dummy/foo.cc{,.orig}

# Reformat and ensure the expected result appeared.
"$dut" drake/dummy/foo.cc
diff -U 999 drake/dummy/foo.cc{,-expected}

# The foo.cc no longer needs reformatting, and check-only leaves it unchanged.
"$dut" --check-only drake/dummy/foo.cc
diff -U 999 drake/dummy/foo.cc{,-expected}

# Done.
echo "PASS"
exit 0
