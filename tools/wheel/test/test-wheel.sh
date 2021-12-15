#!/bin/bash

# This shell script tests a Drake wheel. It must be run inside of a container
# which has been properly provisioned, e.g. by the accompanying test-wheels.sh
# script (in particular, /opt/python must contain a Python virtual environment
# which will be used to run the tests). The wheel must be accessible to the
# container, and the container's path to the wheel should be given as an
# argument to the script.

set -e

if [ -z "$1" ]; then
    echo "Usage: $0 <wheel>" >&2
    exit 1
fi

. /opt/python/bin/activate

pip install --upgrade pip

pip install "$1"

python << EOF
import pydrake.all
print(pydrake.getDrakePath())
print(pydrake.all.PackageMap().GetPath("drake"))
assert pydrake.all.SnoptSolver().available(), "Missing SNOPT"
EOF
