#!/bin/bash
me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
export ASAN_OPTIONS="$ASAN_OPTIONS:check_initialization_order=1:detect_stack_use_after_return=1:suppressions=$mydir/asan.supp"
# LSan is run with ASan by default, ASAN_OPTIONS can't be used to suppress LSan
# errors
export LSAN_OPTIONS="$LSAN_OPTIONS:suppressions=$mydir/lsan.supp"
# Ensure executable named llvm-symbolizer is on the PATH.
export PATH="$PATH:/usr/lib/llvm-6.0/bin:/usr/local/opt/llvm/bin"
"$@"
