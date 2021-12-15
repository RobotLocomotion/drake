#!/bin/bash

# This shell script provisions a bare docker image for testing a Drake wheel.
# It is not intended to be run directly; use test-wheel.sh or test-wheels.sh,
# or the accompanying Dockerfile, instead.

set -e

PYTHON=python${1:-3}

export DEBIAN_FRONTEND=noninteractive

apt-get update

apt-get -y install --no-install-recommends \
    lib${PYTHON}-dev ${PYTHON}-venv \
    python3-venv python3-tk \
    libx11-6 libsm6 libxt6 libglib2.0-0

${PYTHON} -m venv /opt/python
