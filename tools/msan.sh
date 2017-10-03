#!/bin/bash
me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
export MSAN_OPTIONS="$MSAN_OPTIONS:suppressions=$mydir/msan.supp"
"$@"
