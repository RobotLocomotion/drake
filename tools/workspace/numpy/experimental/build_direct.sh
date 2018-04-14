#!/bin/bash
set -e -x -u

# Script to build NumPy directly.

output_dir=${1}
shift

repo=https://github.com/numpy/numpy
commit=pull/10898/head

git clone ${repo} numpy
cd numpy

# Checkout specified commit, accommodating a PR if specified.
if [[ ${commit} =~ ^pull/.*$ ]]; then
    git fetch origin ${commit}
    git checkout FETCH_HEAD
else
    git checkout -f ${commit}
fi

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
    pip install --prefix \${PWD}/tmp ${wheel_file} --ignore-installed

EOF
