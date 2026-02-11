#!/usr/bin/env python

import os
import platform
import sys

from pydrake.all import MosekSolver

# Configure MOSEK to be enabled at runtime (but with broken license).
os.environ["MOSEKLM_LICENSE_FILE"] = "/does/not/exist.lic"
solver = MosekSolver()
assert solver.enabled()

# MOSEK's published wheels declare an upper bound on their supported Python
# version, which is currently Python < 3.15. When that changes to a larger
# version number, we should bump this up to match, and also grep tools/wheel
# for other mentions of MOSEK version bounds and fix those as well.
# Additionally, MOSEK is not currently supported for Linux aarch64 wheels.
# (Apple Silicon is spelled 'arm64', so this doesn't apply there.)
if sys.version_info[:2] < (3, 15) and platform.machine() != "aarch64":
    assert solver.available()
else:
    assert not solver.available()
    sys.exit(0)

# Check that MOSEK lazy loading works. Trying to get a license should trigger
# the lazy loading. MOSEK will raise an error because the license is not valid.
try:
    solver.AcquireLicense()
    message = None
except Exception as e:
    message = str(e)

# Check `e` for text that indicates the _right_ error.
expected = "Could not acquire MOSEK license"
assert expected in message, message
