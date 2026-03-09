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

# Note the "--disable-drake-valgrind-tracing" skip-by-arg option below. Any
# subprocess launched by a test with that as a command-line argument won't
# be instrumented by valgrind. This is useful when you want to instrument
# a C++ unit test but not a helper program that it calls.

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
    --trace-children=yes \
    --trace-children-skip=/bin/cat,/bin/cp,/bin/ln,/bin/ls,/bin/mkdir,/bin/mv,/bin/sed,/lib/ld-linux.so.\*,/lib64/ld-linux-x86-64.so.\*,/usr/bin/clang,/usr/bin/clang-15,/usr/bin/clang-format-15,/usr/bin/diff,/usr/bin/dot,/usr/bin/fc-list,/usr/bin/file,/usr/bin/find,/usr/bin/gcc,/usr/bin/ldd,/usr/bin/strip,/usr/bin/uname,\*/external/buildifier/buildifier \
    --trace-children-skip-by-arg=--disable-drake-valgrind-tracing \
    --track-origins=yes \
    "$@"
