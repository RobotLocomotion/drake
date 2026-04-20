#!/bin/bash

# Internal script to provision a bare Docker image for testing a Drake wheel.

set -eu -o pipefail

readonly PYTHON_VERSION="${1:-3}"
readonly PYTHON_MANAGER="${2:-pip}"

readonly PYTHON="python${PYTHON_VERSION}"
readonly VENV="/tmp/drake-wheel-test/python"

. /etc/os-release

case "${ID}" in
  ubuntu)
    export DEBIAN_FRONTEND=noninteractive
    apt-get -y update
    # Install system prerequisites required by Drake's wheel, only on Ubuntu.
    apt-get -y install --no-install-recommends libx11-6 libsm6 libglib2.0-0t64

    # Install Python and set up the virtual environment.
    case "${PYTHON_MANAGER}" in
      pip)
        apt-get -y install --no-install-recommends \
          lib${PYTHON}-dev ${PYTHON}-venv \
          python3-venv
        ${PYTHON} -m venv ${VENV}
        ;;
      uv)
        apt-get -y install --no-install-recommends \
          ca-certificates gzip tar wget
        wget -qO- https://astral.sh/uv/install.sh | sh
        ${HOME}/.local/bin/uv venv ${VENV} --python ${PYTHON_VERSION}
        ;;
      *)
        echo "Unsupported Python manager '${PYTHON_MANAGER}'" >&2
        exit 1
        ;;
    esac
    ;;
  amzn)
    dnf update -y

    # Install Python and set up the virtual environment.
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
