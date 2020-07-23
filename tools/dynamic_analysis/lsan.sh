#!/bin/bash
me=$(python3 -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
export LSAN_OPTIONS="$LSAN_OPTIONS:strip_path_prefix=/proc/self/cwd/:suppressions=$mydir/lsan.supp"
# Ensure executable named llvm-symbolizer is on the PATH.
export PATH="$PATH:/usr/lib/llvm-9/bin:/usr/local/opt/llvm/bin"
"$@"
