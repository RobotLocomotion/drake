#!/bin/bash
me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
export ASAN_OPTIONS="$ASAN_OPTIONS:check_initialization_order=1:detect_stack_use_after_return=1:suppressions=$mydir/asan.supp"
"$@"
