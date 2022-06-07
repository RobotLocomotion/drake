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

if [ -z "$1" ]; then
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
p = pydrake.solvers.mathematicalprogram.MathematicalProgram()
x = p.NewContinuousVariables(2, "x")
p.AddLinearConstraint(x[0] >= 1)
p.AddLinearConstraint(x[1] >= 1)
p.AddQuadraticCost(numpy.eye(2), numpy.zeros(2), x)
s = pydrake.all.IpoptSolver()
assert s.Solve(p, None, None).is_success(), "IPOPT is not usable"
EOF
