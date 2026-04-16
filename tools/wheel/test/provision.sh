#!/bin/bash

# Internal script to provision a bare Docker image for testing a Drake wheel.

set -eu -o pipefail

readonly PYTHON_VERSION="${1:-3}"
readonly PYTHON_MANAGER="${2:-pip}"

readonly PYTHON="python${PYTHON_VERSION}"
readonly VENV="/tmp/drake-wheel-test/python"

. /etc/os-release

if [[ "${ID}" == "ubuntu" && "${PYTHON_MANAGER}" == "pip" ]]; then
  export DEBIAN_FRONTEND=noninteractive
  apt-get -y update
  # Install system prerequisites required by Drake's wheel, only on Ubuntu.
  apt-get -y install --no-install-recommends libx11-6 libsm6 libglib2.0-0t64

  # Install Python and set up the virtual environment.
  apt-get -y install --no-install-recommends \
    lib${PYTHON}-dev ${PYTHON}-venv \
    python3-venv \
  ${PYTHON} -m venv ${VENV}
  . ${VENV}/bin/activate
  pip install --upgrade pip
elif [[ "${ID}" == "ubuntu" && "${PYTHON_MANAGER}" == "uv" ]]; then
  export DEBIAN_FRONTEND=noninteractive
  apt-get -y update
  # Install system prerequisites required by Drake's wheel, only on Ubuntu.
  apt-get -y install --no-install-recommends libx11-6 libsm6 libglib2.0-0t64

  # Install uv and set up the virtual environment.
  apt-get -y install --no-install-recommends \
    ca-certificates gzip tar wget
  wget -qO- https://astral.sh/uv/install.sh | sh
  ${HOME}/.local/bin/uv venv ${VENV} --python ${PYTHON_VERSION}
elif [[ "${ID}" == "amzn" && "${PYTHON_MANAGER}" == "pip" ]]; then
  dnf update -y --releasever=latest

  # Install Python and set up the virtual environment.
  dnf install -y ${PYTHON}
  ${PYTHON} -m venv ${VENV}
  . ${VENV}/bin/activate
  pip install --upgrade pip
else
  echo "Unsupported distro '${ID}' and/or manager '${PYTHON_MANAGER}'" >&2
  exit 1
fi
