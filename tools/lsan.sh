#!/bin/bash
tools=$(dirname $(realpath "$0"))
export LSAN_OPTIONS="suppressions=$tools/lsan.supp:$LSAN_OPTIONS"
"$@"
