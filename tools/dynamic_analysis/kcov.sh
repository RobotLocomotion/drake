#!/bin/bash

me=$(python3 -c 'import os; print(os.path.realpath("'"$0"'"))')
WORKSPACE=$(dirname $(dirname $(dirname "${me}")))

# There must be ${WORKSPACE}/WORKSPACE.
if [ ! -f "${WORKSPACE}/WORKSPACE" ]; then
  echo "File not found: ${WORKSPACE}/WORKSPACE"
  exit 1
fi

export PATH="/opt/kcov/35/bin:${PATH}"

# ELF binaries with 'sh' in their first 80 bytes are mishandled by kcov,
# leading to segfault. See https://github.com/SimonKagstrom/kcov/issues/339
is_kcov_339_vulnerable () {
    file -b -L "$1" | grep -q ELF \
        && dd bs=80 count=1 if="$1" 2>/dev/null | grep -q sh
}

read -d '' ERROR_339 <<- eof
Error: $1 is vulnerable to kcov issue #339; coverage analysis would
misinterpret the binary as a shell script and then segfault.

The problem affects tests as a result of (essentially) bad luck; some field of
the ELF header happens to contain the ASCII bytes 'sh'. This occurs somewhat
rarely, and is usually a function of the exact program details.

To fix a test program with this problem, make a trivial change (e.g. by
reordering tests), or add "tags = ['no_kcov']" to the test target.
eof

# Avoid tripping kcov issue #339 and give an explicit error. This check can be
# removed once kcov version 40 is in use.
if  is_kcov_339_vulnerable "$1"; then
    echo "$ERROR_339"
    exit 1
fi

kcov \
    "--include-path=${WORKSPACE}" \
    --verify \
    --exclude-pattern=third_party \
    "${TEST_UNDECLARED_OUTPUTS_DIR}/kcov" \
    "--replace-src-path=/proc/self/cwd:${WORKSPACE}" \
    "$@"
