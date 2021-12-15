#!/bin/bash

set -e

PYTHON=python${1:-3}

# Set up Python environment and install Python prerequisites.
apt-get -y install --no-install-recommends \
    ${PYTHON}-dev lib${PYTHON}-dev ${PYTHON}-venv

${PYTHON} -m venv /usr/local

pip install \
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
