#!/bin/bash
tools=$(dirname $(realpath "$0"))
export ASAN_OPTIONS="check_initialization_order=1:detect_stack_use_after_return=1:suppressions=$tools/asan.supp:$ASAN_OPTIONS"
"$@"
