#!/bin/bash
me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")

# Ensure that newly allocated memory that is not directly initialized by GLib
# and memory being freed by GLib are reset to zero.
export G_DEBUG=gc-friendly

# Ensure that all GLib memory slices that are allocated through g_slice_alloc()
# and released by g_slice_free1() are actually allocated via direct calls to
# g_malloc() and g_free().
export G_SLICE=always-malloc

# Disable GTest death test use of clone() because it confuses valgrind.
export GTEST_DEATH_TEST_USE_FORK=1

valgrind \
    --tool=memcheck \
    --gen-suppressions=all \
    --leak-check=full \
    --suppressions="$mydir/valgrind.supp" \
    --suppressions=/usr/lib/valgrind/debian.supp \
    --suppressions=/usr/lib/valgrind/python.supp \
    --error-exitcode=1 \
    --trace-children=yes \
    --trace-children-skip=/bin/cat,/bin/cp,/bin/mkdir,/bin/sed,/usr/bin/clang,/usr/bin/clang-6.0,/usr/bin/clang-format-6.0,/usr/bin/find,/usr/bin/gcc \
    --track-origins=yes \
    --show-leak-kinds=definite,possible \
    --num-callers=16 \
    "$@"
