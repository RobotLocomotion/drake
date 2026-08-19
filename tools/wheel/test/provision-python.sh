#!/bin/bash

# Internal script to provision a Python virtual environment inside an
# otherwise already-provisioned Docker image.

set -eu -o pipefail

readonly PYTHON_VERSION="${1:-3}"
readonly PYTHON_MANAGER="${2:-pip}"

readonly PYTHON="python${PYTHON_VERSION}"
readonly VENV="/tmp/drake-wheel-test/python"

. /etc/os-release

case "${ID}" in
  ubuntu)
    export DEBIAN_FRONTEND=noninteractive
    case "${PYTHON_MANAGER}" in
      pip)
        apt-get -y install --no-install-recommends \
          lib${PYTHON}-dev ${PYTHON}-venv
        ${PYTHON} -m venv ${VENV}
        ;;
      uv)
        ${HOME}/.local/bin/uv venv ${VENV} --python ${PYTHON_VERSION}
        ;;
      *)
        echo "Unsupported Python manager '${PYTHON_MANAGER}'" >&2
        exit 1
        ;;
    esac
    ;;
  amzn)
    case "${PYTHON_MANAGER}" in
      pip)
        dnf install -y ${PYTHON}
        ${PYTHON} -m venv ${VENV}
        ;;
      *)
        echo "Unsupported Python manager '${PYTHON_MANAGER}'" >&2
        exit 1
        ;;
    esac
    ;;
  *)
    echo "Unsupported distro '${ID}'" >&2
    exit 1
    ;;
esac
