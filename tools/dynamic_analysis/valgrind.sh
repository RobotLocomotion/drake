#!/bin/bash
me=$(python3 -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "${me}")

# Ensure that newly allocated memory that is not directly initialized by GLib
# and memory being freed by GLib are reset to zero.
export G_DEBUG=gc-friendly

# Ensure that all GLib memory slices that are allocated through g_slice_alloc()
# and released by g_slice_free1() are actually allocated via direct calls to
# g_malloc() and g_free().
export G_SLICE=always-malloc

# Ensure that Google Test uses _exit() or fork() instead of clone() in death
# tests.
export GTEST_DEATH_TEST_USE_FORK=1

# Set to empty so that tests can rely on the presence of this variable to check
# if they're running under valgrind.
export VALGRIND_OPTS=""

valgrind \
    --error-exitcode=1 \
    --gen-suppressions=all \
    --leak-check=full \
    --num-callers=16 \
    --show-leak-kinds=definite,possible \
    --suppressions="${mydir}/valgrind.supp" \
    --suppressions=/usr/lib/valgrind/debian.supp \
    --suppressions=/usr/lib/valgrind/python3.supp \
    --tool=memcheck \
    --track-origins=yes \
    "$@"
