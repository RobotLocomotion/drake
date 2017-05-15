#!/bin/bash
tools=$(dirname $(realpath "$0"))
export UBSAN_OPTIONS="$UBSAN_OPTIONS:suppressions=$tools/ubsan.supp"
"$@"
