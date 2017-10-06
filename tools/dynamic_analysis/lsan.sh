#!/bin/bash
me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
export LSAN_OPTIONS="$LSAN_OPTIONS:suppressions=$mydir/lsan.supp"
"$@"
