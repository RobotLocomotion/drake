#!/bin/bash
me=$(python3 -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
export ASAN_OPTIONS="$ASAN_OPTIONS:check_initialization_order=1:detect_container_overflow=0:detect_invalid_pointer_pairs=1:detect_stack_use_after_return=1:strict_init_order=1:strict_string_checks=1:strip_path_prefix=/proc/self/cwd/:suppressions=$mydir/asan.supp"
# LSan is run with ASan by default, ASAN_OPTIONS can't be used to suppress LSan
# errors
export LSAN_OPTIONS="$LSAN_OPTIONS:strip_path_prefix=/proc/self/cwd/:suppressions=$mydir/lsan.supp"
# Ensure executable named llvm-symbolizer is on the PATH.
export PATH="$PATH:/usr/lib/llvm-9/bin:/usr/local/opt/llvm/bin"
"$@"
