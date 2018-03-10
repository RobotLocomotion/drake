#!/bin/bash


me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
WORKSPACE=$(dirname $(dirname $(dirname "$me")))

# There must be ${WORKSPACE}/WORKSPACE.
if [ ! -f "${WORKSPACE}/WORKSPACE" ]; then
  echo "File not found: ${WORKSPACE}/WORKSPACE"
  exit 1
fi

# There must be kcov (it is declared as a "data = []" dependency for ":kcov"
# in our BUILD file).
KCOV="${PWD}/external/kcov/kcov"
if [ ! -x "${KCOV}" ]; then
  echo "Kcov binary not found at ${kcov}"
  exit 1
fi

"$KCOV" \
    --include-path=$WORKSPACE \
    --exclude-pattern=third_party \
    $WORKSPACE/bazel-kcov \
    --replace-src-path=/proc/self/cwd:$WORKSPACE \
    "$@"
