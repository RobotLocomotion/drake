#!/bin/bash -vx


me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
WORKSPACE=$(dirname $(dirname "$me"))

# Just in case, don't collect data from cpplint and _pycodestyle tests.
if echo "$@" | grep -q "_cpplint\|_pycodestyle" ; then
    "$@"
    exit $?
fi

kcov \
    --include-path=$WORKSPACE \
    --exclude-pattern=thirdParty,externals \
    $WORKSPACE/bazel-kcov \
    "$@"
