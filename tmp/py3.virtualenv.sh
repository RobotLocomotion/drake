#!/bin/bash

_dir=$(dirname ${BASH_SOURCE})

if [[ ! -f ${_dir}/virtualenv/bin/python3 ]]; then
(
    set -x
    cd ${_dir}
    python3 -m virtualenv --python python3 virtualenv
    set +x
    source virtualenv/bin/activate
    # Install some (if not all) needed dependencies.
    pip install -I \
        pyyaml protobuf==3.6.0 sphinx==1.8.1 sphinx_rtd_theme \
        numpy==1.14.0 zmq tornado matplotlib pydot
    set -x
    # Reflect system Python; make `python` and `python-config` fall through to
    # system, and use `python{major}-config`.
    rm virtualenv/bin/python
    mv virtualenv/bin/{python-config,python3-config}
    sed -i 's#py3/bin/python\b#py3/bin/python3#g' virtualenv/bin/python3-config
)
fi

source ${_dir}/virtualenv/bin/activate
