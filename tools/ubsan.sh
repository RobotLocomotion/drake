#!/bin/bash
me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
tools=$(dirname "$me")
# halt_on_error exits the program when the *first* error is detected by UBSan.
# We're adding this because `exitcode` set in `UBSAN_OPTIONS` is not heeded by
# clang without this option.
export UBSAN_OPTIONS="$UBSAN_OPTIONS:
suppressions=$tools/ubsan.supp:
halt_on_error=1"
"$@"
