#!/bin/bash

# Internal script to provision a bare Docker image for testing a Drake wheel.

set -eu -o pipefail

# Parse our arguments.
readonly PYTHON_VERSION=${1:-3}
readonly PYTHON="python${PYTHON_VERSION}"
readonly PYTHON_MANAGER=${2:-pip}

# Set up platform-specific installation commands and packages.
readonly UV_PREREQS=(ca-certificates gzip tar wget)
INSTALL_COMMAND=
PYTHON_PACKAGES=

. /etc/os-release
case "$ID" in
  ubuntu)
    export DEBIAN_FRONTEND=noninteractive
    apt-get -y update
    INSTALL_COMMAND=(apt-get -y install --no-install-recommends)
    PYTHON_PACKAGES=(lib${PYTHON}-dev ${PYTHON}-venv python3-venv)
    # Install system prerequisites required by Drake's wheel, only on Ubuntu.
    "${INSTALL_COMMAND[@]}" libx11-6 libsm6 libglib2.0-0t64
    ;;
  amzn)
    dnf update -y --releasever=latest
    INSTALL_COMMAND=(dnf install -y)
    PYTHON_PACKAGES=(${PYTHON})
    ;;
  *)
    echo "Unknown distro '$ID'" >&2
    exit 1
    ;;
esac

# Install Python and set up the virtual environment...
readonly VENV="/tmp/drake-wheel-test/python"
if [[ "${PYTHON_MANAGER}" == "uv" ]]; then
  # ...using uv.
  "${INSTALL_COMMAND[@]}" "${UV_PREREQS[@]}"
  wget -qO- https://astral.sh/uv/install.sh | sh
  ${HOME}/.local/bin/uv venv ${VENV} --python ${PYTHON_VERSION}
else
  # ...using the host-system Python.
  "${INSTALL_COMMAND[@]}" "${PYTHON_PACKAGES[@]}"
  ${PYTHON} -m venv ${VENV}
  . ${VENV}/bin/activate
  pip install --upgrade pip
fi
