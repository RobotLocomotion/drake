#!/bin/bash
tools=$(dirname $(realpath "$0"))
export TSAN_OPTIONS="detect_deadlocks=1:second_deadlock_stack=1:suppressions=$tools/tsan.supp:$TSAN_OPTIONS"
"$@"
