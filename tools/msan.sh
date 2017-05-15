#!/bin/bash
me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
tools=$(dirname "$me")
export MSAN_OPTIONS="$MSAN_OPTIONS:suppressions=$tools/msan.supp"
"$@"
