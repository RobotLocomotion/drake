#!/bin/bash
me=$(python3 -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
# halt_on_error exits the program when the *first* error is detected by UBSan.
# We're adding this because `exitcode` set in `UBSAN_OPTIONS` is not heeded by
# clang without this option.
export UBSAN_OPTIONS="$UBSAN_OPTIONS:halt_on_error=1:strip_path_prefix=/proc/self/cwd/:suppressions=$mydir/ubsan.supp"
# Ensure executable named llvm-symbolizer is on the PATH.
export PATH="$PATH:/usr/lib/llvm-9/bin:/usr/local/opt/llvm/bin"
"$@"
