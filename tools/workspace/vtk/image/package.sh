#!/bin/bash

set -eu -o pipefail

# Get various version numbers.
readonly vtk_tag=vtk-9.2.0
# To re-package, increase build_number by 1 and update repository.bzl to avoid
# overwriting artifacts thus breaking historical builds.
readonly build_number=1
readonly platform=$(lsb_release --codename --short)-$(uname --processor)

# Create archive named:
#   vtk-<version>
#     -<build_number>
#     -<distribution codename>
#     -<processor architecture>.tar.gz
readonly archive=${vtk_tag}-${build_number}-${platform}.tar.gz
cd /opt/vtk

tar czf "${archive}" -- *
shasum --algorithm 256 "${archive}" | tee "${archive}.sha256"
