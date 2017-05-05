#!/bin/bash
tools=$(dirname $(realpath "$0"))
export UBSAN_OPTIONS="suppressions=$tools/ubsan.supp:$UBSAN_OPTIONS"
"$@"
