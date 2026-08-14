#!/bin/bash

# Internal script to install Python and required Python packages.
# Docker (Linux) only.

set -eu -o pipefail

readonly PYTHON_VERSION="${1-}"

if [ -z "${PYTHON_VERSION}" ]; then
  echo "Usage: $0 <python version>" >&2
  exit 1
fi
if [[ ! "$PYTHON_VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "Expected Python version in the form 'x.y.z'" >&2
  exit 1
fi

readonly PYTHON="python${PYTHON_VERSION%.*}"

wget -qO- https://astral.sh/uv/install.sh | sh
${HOME}/.local/bin/uv venv --allow-existing /usr/local --python ${PYTHON}
readonly PREFIX=$(realpath $(dirname $(${HOME}/.local/bin/uv python find ${PYTHON}))/../)

ln -s ${PREFIX}/bin/${PYTHON}-config /usr/bin/python3-config
ln -s ${PREFIX}/bin/${PYTHON}-config /usr/local/bin/python-config
ln -s ${PREFIX}/include/${PYTHON} /usr/local/include/
ln -s /usr/local/bin/python /usr/bin/python
