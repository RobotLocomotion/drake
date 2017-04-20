#!/bin/bash

# Disable GTest death test use of clone() because it confuses valgrind.
set -x;
export GTEST_DEATH_TEST_USE_FORK=1

# Only run the test when --no_memcheck is not passed
for args in "$@"; do
    if [[ "$args" == "--no_memcheck" ]]; then
        exit
    fi
done

valgrind \
  --leak-check=full \
  --suppressions="tools/valgrind.supp" \
  --error-exitcode=1 \
  --trace-children=yes \
  --track-origins=yes \
  --show-leak-kinds=definite,possible \
  "$@";
