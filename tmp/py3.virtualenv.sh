#!/bin/bash

_env_dir=$(cd $(dirname ${BASH_SOURCE}) && pwd)/virtualenv
if [[ ! -f ${_env_dir}/bin/python3 ]]; then
(
    set -eux
    cd $(dirname ${_env_dir})
    python3_bin=$(which python3)
    ${python3_bin} -m virtualenv --python ${python3_bin} ${_env_dir}
    set +eux
    source ${_env_dir}/bin/activate
    # Install some (if not all) needed dependencies.
    pip install -r ${_env_dir}/../py3.requirements.txt
    set -eux
    # Reflect system Python; make `python` and `python-config` fall through to
    # system, and use `python{major}-config`.
    rm ${_env_dir}/bin/python
    mv ${_env_dir}/bin/{python-config,python3-config}
    sed -i 's#/bin/python\b#/bin/python3#g' ${_env_dir}/bin/python3-config
)
fi

source ${_env_dir}/bin/activate
unset _env_dir
