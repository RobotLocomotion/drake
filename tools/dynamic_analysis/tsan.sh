#!/bin/bash
me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
export TSAN_OPTIONS="$TSAN_OPTIONS:detect_deadlocks=1:second_deadlock_stack=1:suppressions=$mydir/tsan.supp"
"$@"
