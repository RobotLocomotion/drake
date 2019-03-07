#!/bin/bash


me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
WORKSPACE=$(dirname $(dirname $(dirname "${me}")))

# There must be ${WORKSPACE}/WORKSPACE.
if [ ! -f "${WORKSPACE}/WORKSPACE" ]; then
  echo "File not found: ${WORKSPACE}/WORKSPACE"
  exit 1
fi

export PATH="/opt/kcov/35/bin:${PATH}"

kcov \
    "--include-path=${WORKSPACE}" \
    --verify \
    --exclude-pattern=third_party \
    "${WORKSPACE}/bazel-kcov" \
    "--replace-src-path=/proc/self/cwd:${WORKSPACE}" \
    "$@"
