#!/bin/bash
tools=$(dirname $(realpath "$0"))
export MSAN_OPTIONS="suppressions=$tools/msan.supp:$MSAN_OPTIONS"
"$@"
