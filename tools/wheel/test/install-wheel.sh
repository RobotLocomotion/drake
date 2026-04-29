#!/bin/bash

# This shell script installs a Drake wheel. It must be run inside a container
# (Ubuntu) or environment (macOS) which has been properly provisioned, e.g. by
# the accompanying Dockerfile (Ubuntu) or by macos/provision-test-python.sh
# (macOS). In particular, /tmp/drake-wheel-test/python must contain a Python
# virtual environment which will be used to run the tests. The path to the
# wheel to be installed must be given as an argument to the script. On Ubuntu,
# the wheel must be accessible to the container, and the path must be the
# container's path to the wheel (rather than the host's path).
#
# In general, it is not recommended to attempt to run this script directly;
# use //tools/wheel:builder instead.

set -eu -o pipefail

WHEEL="${1:-}"
PYTHON_MANAGER="${2:-pip}"

if [[ -z "${WHEEL}" ]]; then
    echo "Usage: $0 <wheel> [{pip,uv}]" >&2
    exit 1
fi

. /tmp/drake-wheel-test/python/bin/activate

case "${PYTHON_MANAGER}" in
    pip)
        pip install "${WHEEL}"
        ;;
    uv)
        ${HOME}/.local/bin/uv pip install "${WHEEL}"
        ;;
    *)
        echo "Unknown virtual environment manager '${PYTHON_MANAGER}'" >&2
        exit 1
        ;;
esac
