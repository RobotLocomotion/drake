#!/bin/bash -vx

WORKSPACE=$(dirname $(dirname $(readlink -f $0)))

# Just in case, don't collect data from cpplint tests.
if echo "$@" | grep -q _cpplint ; then
    "$@"
    exit $?
fi

kcov \
    --include-path=$WORKSPACE \
    --exclude-pattern=thirdParty,externals \
    $WORKSPACE/bazel-kcov \
    "$@"
