#!/bin/bash -e

# This shell script tests a Drake wheel. It is intended to be run inside either
# a "pristine" Ubuntu container, or one which has been provisioned by the
# accompanying test-wheels.sh script. The wheel must be accessible to the
# container, and the container's path to the wheel should be given as an
# argument to the script. If no path is specified, "drake" from PyPI will be
# tested.

if [ ! -d /opt/python ]; then
    tr="$(readlink -f "$(dirname "${BASH_SOURCE}")")"
    "$tr/provision.sh" $2
fi

. /opt/python/bin/activate

pip install --upgrade pip

pip install "${1:-drake}"

python << EOF
import pydrake.all
print(pydrake.getDrakePath())
EOF
