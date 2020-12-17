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

# If unset, set to empty string.
export VALGRIND_OPTS="$VALGRIND_OPTS"

valgrind \
    --error-exitcode=1 \
    --gen-suppressions=all \
    --num-callers=16 \
    --suppressions="${mydir}/helgrind.supp" \
    --suppressions=/usr/lib/valgrind/debian.supp \
    --suppressions=/usr/lib/valgrind/python.supp \
    --tool=helgrind \
    --trace-children=yes \
    --trace-children-skip=/bin/cat,/bin/cp,/bin/ln,/bin/ls,/bin/mkdir,/bin/mv,/bin/sed,/lib/ld-linux.so.\*,/lib64/ld-linux-x86-64.so.\*,/usr/bin/clang,/usr/bin/clang-9,/usr/bin/clang-format-9,/usr/bin/diff,/usr/bin/dot,/usr/bin/fc-list,/usr/bin/file,/usr/bin/find,/usr/bin/gcc,/usr/bin/ldd,/usr/bin/patchelf,/usr/bin/strip,/usr/bin/uname,\*/external/buildifier/buildifier \
    --trace-children-skip-by-arg=meshcat.servers.zmqserver,uname \
    "$@"
