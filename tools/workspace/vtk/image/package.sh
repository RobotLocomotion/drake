#!/bin/bash

set -eu -o pipefail

readonly PYTHON_VERSINFO=($(python3 -c 'import sys; print(*sys.version_info)'))
readonly python=python${PYTHON_VERSINFO[0]}.${PYTHON_VERSINFO[1]}

# Get various version numbers.
readonly vtk_tag=vtk-9.1.0
readonly py_tag=python-$(echo ${PYTHON_VERSINFO[@]:0:3} | tr ' ' '.')
readonly platform=$(lsb_release --codename --short)-$(uname --processor)

# Create archive named:
#   vtk-<version>
#     -python-<python version>
#     -<distribution codename>
#     -<processor architecture>.tar.gz
readonly archive=${vtk_tag}-${py_tag}-${platform}.tar.gz
cd /opt/vtk

tar czf $archive -- *
shasum --algorithm 256 $archive | tee $archive.sha256
