#!/bin/bash

# Internal script to provision a bare Docker image for testing a Drake wheel.

set -eu -o pipefail

readonly PYTHON=python${1:-3}

. /etc/os-release
case "$ID" in
  ubuntu)
    export DEBIAN_FRONTEND=noninteractive
    apt-get -y update
    apt-get -y install --no-install-recommends \
        lib${PYTHON}-dev ${PYTHON}-venv \
        python3-venv \
        libx11-6 libsm6 libxt6 libglib2.0-0
    ;;
  amzn)
    dnf install -y ${PYTHON}
    ;;
  *)
    echo "Unknown distro '$ID'" >&2
    exit 1
    ;;
esac

${PYTHON} -m venv /tmp/drake-wheel-test/python

. /tmp/drake-wheel-test/python/bin/activate

pip install --upgrade pip
