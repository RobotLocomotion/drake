#!/bin/bash
me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
tools=$(dirname "$me")
export UBSAN_OPTIONS="$UBSAN_OPTIONS:suppressions=$tools/ubsan.supp"
"$@"
