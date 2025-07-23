#!/bin/bash

me=$(python3 -c 'import os; print(os.path.realpath("'"$0"'"))')
WORKSPACE=$(dirname $(dirname $(dirname "${me}")))

# There must be ${WORKSPACE}/WORKSPACE.
if [ ! -f "${WORKSPACE}/WORKSPACE" ]; then
  echo "File not found: ${WORKSPACE}/WORKSPACE"
  exit 1
fi

read -d '' DRAKE_KCOV_COMMAND << eof
kcov \
    "--include-path=${WORKSPACE}" \
    --verify \
    --python-parse=python3 \
    --exclude-pattern=third_party \
    "--replace-src-path=/proc/self/cwd:${WORKSPACE}" \
    "${TEST_UNDECLARED_OUTPUTS_DIR}/kcov"
eof

# If we have a Python target...
if [[ $(file -b --mime -L "$1") == *text/x-*python* ]]; then
    # Bazel's Python wrappers[1] require extra treatment. PYTHON_COVERAGE and
    # COVERAGE_DIR are consumed by the wrapper. The wrapper will try to write
    # to COVERAGE_DIR, so it needs to be created here first.
    # [1]: https://github.com/bazelbuild/bazel/blob/6.0.0/src/main/java/com/google/devtools/build/lib/bazel/rules/python/python_stub_template.txt
    export PYTHON_COVERAGE="${WORKSPACE}/tools/dynamic_analysis/kcoverage.py"
    export COVERAGE_DIR="${TEST_UNDECLARED_OUTPUTS_DIR}/kcov"
    mkdir -p ${COVERAGE_DIR}

    # DRAKE_KCOV_COMMAND is consumed by drake's kcov/python integration.
    export DRAKE_KCOV_COMMAND

    # Execute our "target", which is really the Bazel Python wrapper.
    exec "$@"
fi

eval ${DRAKE_KCOV_COMMAND} "$@"
