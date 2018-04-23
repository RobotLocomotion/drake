#!/bin/bash
set -eux -o pipefail

# Script to build NumPy directly. Should only be called by `build_ubuntu.sh` or
# `build_mac.sh`.

output_dir=${1}
shift

repo=https://github.com/EricCousineau-TRI/numpy
# Requires NumPy PR #10898 and #11076
commit=8b517bd2ccf37dd51191052e2b39be4f92d42bca
fetch_ref=drake_tmp

git clone ${repo} numpy
cd numpy
git fetch origin ${fetch_ref}
git checkout -f ${commit}

# Build NumPy, record the generated wheel file (assuming there is only one).
python setup.py bdist_wheel "$@"
wheel_file=$(echo ${PWD}/dist/numpy*.whl)

# Briefly print out NumPy version:
python -m virtualenv env
cd env
set +e +x +u
source bin/activate
set -e -x -u
python -m pip install ${wheel_file}
python -c 'import numpy; print(numpy.version.full_version)'

# Copy to directory, which should be a mounted volume.
cp ${wheel_file} ${output_dir}

cat <<EOF

NumPy wheel file built: ${wheel_file}

As an example to install to a prefix to manually test:
    pip install --prefix \${PWD}/tmp $(basename ${wheel_file}) --ignore-installed

EOF
