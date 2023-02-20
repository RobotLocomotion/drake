#!/bin/bash
if [[ ${0} != ${BASH_SOURCE} ]]; then
    echo "Please execute, don't source." >&2
    return 1
fi
set -eu

cd $(dirname ${BASH_SOURCE})
if [[ -d ./venv ]]; then
    exit 0
fi

python -m venv ./venv
./venv/bin/pip install -U pip wheel
./venv/bin/pip install -r venv.requirements.txt
./venv/bin/pip freeze > venv.requirements.freeze.txt
