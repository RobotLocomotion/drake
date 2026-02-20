#!/bin/bash
me=$(python3 -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
export TSAN_OPTIONS="detect_deadlocks=1:second_deadlock_stack=1:strip_path_prefix=/proc/self/cwd/:suppressions=$mydir/tsan.supp"
# Ensure executable named llvm-symbolizer is on the PATH.
export PATH="$PATH:/usr/lib/llvm-19/bin:/usr/lib/llvm-15/bin:/usr/local/opt/llvm/bin"
# Run with ASLR disabled.
setarch $(uname -m) -R "$@"
