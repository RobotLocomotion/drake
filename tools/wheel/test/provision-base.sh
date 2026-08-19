#!/bin/bash

# Internal script to prepare a bare Docker image to be provisioned for testing
# a Drake wheel.

set -eu -o pipefail

readonly PYTHON_MANAGER="${1:-pip}"

. /etc/os-release

case "${ID}" in
  ubuntu)
    export DEBIAN_FRONTEND=noninteractive
    apt-get -y update
    # Install system prerequisites required by Drake's wheel, only on Ubuntu.
    apt-get -y install --no-install-recommends libx11-6 libsm6 libglib2.0-0t64

    case "${PYTHON_MANAGER}" in
      pip)
        apt-get -y install --no-install-recommends python3-venv
        ;;
      uv)
        apt-get -y install --no-install-recommends \
          ca-certificates gzip tar wget
        wget -qO- https://astral.sh/uv/install.sh | sh
        ;;
      *)
        echo "Unsupported Python manager '${PYTHON_MANAGER}'" >&2
        exit 1
        ;;
    esac
    ;;
  amzn)
    dnf update -y
    # N.B. AL2023 provides all the Python versions we need via dnf, so we
    # don't need, nor support, uv here, and we install a specific version in
    # provision-python.sh.
    ;;
  *)
    echo "Unsupported distro '${ID}'" >&2
    exit 1
    ;;
esac
