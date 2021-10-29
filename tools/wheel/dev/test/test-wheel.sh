#!/bin/bash -e

# This shell script tests a Drake wheel. It is intended to be run inside a
# "pristine" Ubuntu container. The wheel must be accessible to the container,
# and the container's path to the wheel should be given as an argument to the
# script. If no path is specified, "drake" from PyPI will be tested.

PYTHON=python${2:-3}

export DEBIAN_FRONTEND=noninteractive

apt-get update

apt-get -y install --no-install-recommends \
    lib${PYTHON}-dev ${PYTHON}-venv \
    python3-venv python3-tk \
    libx11-6 libsm6 libxt6 libglib2.0-0

${PYTHON} -m venv /opt/python

. /opt/python/bin/activate

pip install --upgrade pip

pip install "${1:-drake}"

python << EOF
import pydrake.all
print(pydrake.getDrakePath())
EOF
