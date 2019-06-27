#!/bin/bash
me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
export LSAN_OPTIONS="$LSAN_OPTIONS:suppressions=$mydir/lsan.supp"
# Ensure executable named llvm-symbolizer is on the PATH.
export PATH="$PATH:/usr/lib/llvm-6.0/bin:/usr/local/opt/llvm/bin"
"$@"
