#!/bin/bash

# Internal script to install Python and required Python packages.
# Docker (Linux) only.

set -eu -o pipefail

if [ "${1%:*}" == "build" ]; then
    readonly PREFIX=/opt/drake-python
    readonly PYTHON=python$(echo ${1#*:} | cut -d. -f1-2)

    cd "$(dirname "${BASH_SOURCE}")"
    ./build-python.sh ${1#*:} ${PREFIX} $2
else
    readonly PREFIX=/usr
    readonly PYTHON=python${1:-3}

    # Set up Python environment and install Python prerequisites.
    apt-get -y update
    apt-get -y install --no-install-recommends \
        ${PYTHON}-dev lib${PYTHON}-dev ${PYTHON}-venv
fi

${PREFIX}/bin/${PYTHON} -m venv /usr/local

ln -s ${PREFIX}/bin/${PYTHON}-config /usr/bin/python3-config
ln -s ${PREFIX}/bin/${PYTHON}-config /usr/local/bin/python-config
ln -s ${PREFIX}/include/${PYTHON} /usr/local/include/
ln -s ${PREFIX}/include/${PYTHON}m /usr/local/include/
ln -s /usr/local/bin/python /usr/bin/python

pip install \
    lxml \
    pyyaml \
    semantic-version \
    setuptools \
    wheel \
    auditwheel
