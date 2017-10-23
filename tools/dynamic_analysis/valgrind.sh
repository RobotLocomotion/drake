#!/bin/bash
me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")

# Disable GTest death test use of clone() because it confuses valgrind.
export GTEST_DEATH_TEST_USE_FORK=1

valgrind \
    --leak-check=full \
    --suppressions="$mydir/valgrind.supp" \
    --error-exitcode=1 \
    --trace-children=yes \
    --track-origins=yes \
    --show-leak-kinds=definite,possible \
    --num-callers=16 \
    "$@"
