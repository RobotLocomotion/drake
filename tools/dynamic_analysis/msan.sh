#!/bin/bash
me=$(python3 -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
export MSAN_OPTIONS="$MSAN_OPTIONS:strip_path_prefix=/proc/self/cwd/:suppressions=$mydir/msan.supp"
"$@"
