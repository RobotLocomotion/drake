#!/bin/bash

# This shell script tests a Drake wheel. It must be run inside of a container
# (Ubuntu) or environment (macOS) which has been properly provisioned, e.g. by
# the accompanying Dockerfile (Ubuntu) or by macos/test-wheel.sh (macOS). In
# particular, /opt/drake-wheel-test/python must contain a Python virtual
# environment which will be used to run the tests. The path to the wheel to be
# tested must be given as an argument to the script. On Ubuntu, the wheel must
# be accessible to the container, and the path must be the container's path to
# the wheel (rather than the host's path).
#
# In general, it is not recommended to attempt to run this script directly;
# use //tools/wheel:builder instead.

set -eu -o pipefail

if [[ -z "$1" ]]; then
    echo "Usage: $0 <wheel>" >&2
    exit 1
fi

. /opt/drake-wheel-test/python/bin/activate

pip install "$1"

python << EOF
import numpy
import pydrake.all

# Basic sanity checks.
print(pydrake.getDrakePath())
print(pydrake.all.PackageMap().GetPath("drake"))

# Check for presence of optional solvers.
assert pydrake.all.MosekSolver().available(), "Missing MOSEK"
assert pydrake.all.SnoptSolver().available(), "Missing SNOPT"

# Check that IPOPT is working.
prog = pydrake.all.MathematicalProgram()
x = prog.NewContinuousVariables(2, "x")
prog.AddLinearConstraint(x[0] >= 1)
prog.AddLinearConstraint(x[1] >= 1)
prog.AddQuadraticCost(numpy.eye(2), numpy.zeros(2), x)
solver = pydrake.all.IpoptSolver()
assert solver.Solve(prog, None, None).is_success(), "IPOPT is not usable"
EOF

python - "$1" << EOF
import os
import sys

# Check that wheel size is within PyPI's per-wheel size quota.
wheel_size = os.path.getsize(sys.argv[1])
wheel_size_in_mib = wheel_size / float(1 << 20)
pypi_wheel_max_size = 100 << 20  # 100 MiB
fail_message = f"Wheel is too large ({wheel_size_in_mib:.2f} MiB) for PyPI"
assert wheel_size < pypi_wheel_max_size, fail_message
EOF

python - "$1" << EOF
import os
import pydrake

def assert_exists(path):
    assert os.path.exists(path), f"{path!r} does not exist!"

# Check that type information files are present.
pydrake_dir = pydrake.__path__[0]
assert_exists(os.path.join(pydrake_dir, 'py.typed'))
assert_exists(os.path.join(pydrake_dir, 'all.pyi'))
EOF
