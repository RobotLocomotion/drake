#!/bin/bash

# Internal script to install Python and required Python packages.
# Docker (Linux) only.

set -eu -o pipefail

readonly PYTHON=python${1:-3}

# Set up Python environment and install Python prerequisites.
apt-get -y update
apt-get -y install --no-install-recommends \
    ${PYTHON}-dev lib${PYTHON}-dev ${PYTHON}-venv

${PYTHON} -m venv /usr/local

# TODO(jwnimmer-tri): Should these be version-pinned? What's the process for
# keeping them up to date if they are?
pip install \
    lxml \
    matplotlib \
    numpy \
    pyyaml \
    semantic-version \
    setuptools \
    wheel \
    auditwheel

ln -s /usr/bin/${PYTHON}-config /usr/bin/python3-config
ln -s /usr/bin/${PYTHON}-config /usr/local/bin/python-config
ln -s /usr/local/bin/python /usr/bin/python
ln -s /usr/include/${PYTHON} /usr/local/include/
ln -s /usr/include/${PYTHON}m /usr/local/include/
