#!/bin/bash

# This test should only be run via Bazel, never directly on the command line.
# Unit test for lcm_vector_gen.py.

set -ex

[[ -n "${TEST_TMPDIR}" ]]  # Fail-fast when not under Bazel.

# Dump some debugging output.
find .

# These are the filenames under test.
tool_outputs="lcmt_sample_t.lcm sample.cc sample.h"

# Insist that the generated output matches the goal copy in git.
mv tools/vector_gen/test/lcmt_sample_t.lcm tools/vector_gen/test/gen/
for item in $tool_outputs; do
  # Tweak the magic "generated file" marker, so that changes to the goal files
  # will show up during code review.
  sed -e 's#GENERATED FILE#GENERATED GOAL#' \
    tools/vector_gen/test/gen/"${item}" > \
    "${TEST_TMPDIR}"/"${item}".generated

  # Compare the tweaked generated file with the goal file from the source tree.
  diff --unified=20 \
    tools/vector_gen/test/goal/"${item}" \
    "${TEST_TMPDIR}"/"${item}".generated
done

echo "PASS"
