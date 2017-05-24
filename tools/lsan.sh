#!/bin/bash
me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
tools=$(dirname "$me")
export LSAN_OPTIONS="$LSAN_OPTIONS:suppressions=$tools/lsan.supp"
"$@"
