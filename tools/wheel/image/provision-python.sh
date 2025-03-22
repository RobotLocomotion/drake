#!/bin/bash

# Internal script to install Python and required Python packages.
# Docker (Linux) only.

set -eu -o pipefail

readonly PREFIX=/opt/drake-python

# The Python version is either e.g. '3.10' or 'build:3.10.0' (or is not
# specified). The 'build:3.x.y' form indicates that we should build our own
# Python rather than using system packages, and requires a complete version
# (needed to download the sources). Otherwise, the version should be a valid
# suffix of 'python'. Unspecified is treated as '3'.
if [[ "${1%:*}" == "build" ]]; then
    readonly SRC=${PREFIX}
    readonly PYTHON=python$(echo ${1#*:} | cut -d. -f1-2)

    cd "$(dirname "${BASH_SOURCE}")"
    ./build-python.sh ${1#*:} ${SRC} $2

    ln -s ${SRC}/bin/python3 ${PREFIX}/bin/python
else
    readonly SRC=/usr
    readonly PYTHON=python${1:-3}

    # Set up Python environment and install Python prerequisites.
    apt-get -y update
    apt-get -y install --no-install-recommends \
        ${PYTHON}-dev lib${PYTHON}-dev ${PYTHON}-venv

    mkdir ${PREFIX}
    ${SRC}/bin/${PYTHON} -m venv ${PREFIX}

    # Python 3.11 venv creates an empty directory here, which prevents us
    # creating a symlink to the real directory, so remove it if present.
    rmdir ${PREFIX}/include/${PYTHON} || true

    ln -s ${SRC}/bin/${PYTHON}-config ${PREFIX}/bin/python3-config
    ln -s ${SRC}/include/${PYTHON} ${PREFIX}/include/
    ln -s ${SRC}/include/${PYTHON}m ${PREFIX}/include/
fi

ln -s ${SRC}/bin/${PYTHON}-config ${PREFIX}/bin/python-config
ln -s ${PREFIX}/bin/python3 /usr/bin/python
