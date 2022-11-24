#!/bin/bash

# Internal script to install Python and required Python packages.
# Docker (Linux) only.

set -eu -o pipefail

# The Python version is either e.g. '3.10' or 'build:3.10.0' (or is not
# specified). The 'build:3.x.y' form indicates that we should build our own
# Python rather than using system packages, and requires a complete version
# (needed to download the sources). Otherwise, the version should be a valid
# suffix of 'python'. Unspecified is treated as '3'.
if [[ "${1%:*}" == "build" ]]; then
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

# Python 3.11 venv creates an empty directory here, which prevents us creating
# a symlink to the real directory, so remove it if present.
rmdir /usr/local/include/${PYTHON} || true

ln -s ${PREFIX}/bin/${PYTHON}-config /usr/bin/python3-config
ln -s ${PREFIX}/bin/${PYTHON}-config /usr/local/bin/python-config
ln -s ${PREFIX}/include/${PYTHON} /usr/local/include/
ln -s ${PREFIX}/include/${PYTHON}m /usr/local/include/
ln -s /usr/local/bin/python /usr/bin/python

# TODO(jwnimmer-tri): Should these be version-pinned? What's the process for
# keeping them up to date if they are?
pip install \
    matplotlib \
    numpy \
    pyyaml \
    semantic-version \
    setuptools \
    wheel \
    auditwheel

if [[ "$(uname)" == "Linux" ]]; then
    pip install patchelf
fi
