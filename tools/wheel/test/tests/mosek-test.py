#!/usr/bin/env python3

import os
import platform
import sys

from pydrake.all import MosekSolver

# Configure MOSEK to be enabled at runtime (but with broken license).
os.environ["MOSEKLM_LICENSE_FILE"] = "/does/not/exist.lic"
solver = MosekSolver()

# MOSEK's published wheels declare an upper bound on their supported Python
# version, which is currently Python < 3.15. When that changes to a larger
# version number, we should bump this up to match, and also grep tools/wheel
# for other mentions of MOSEK version bounds and fix those as well.
# Additionally, MOSEK is not currently supported for Linux aarch64 wheels.
# (Apple Silicon is spelled 'arm64', so this doesn't apply there.)
if platform.machine() == "aarch64":
    assert not solver.available()  # Disabled at build-time.
    assert solver.enabled()  # License environment variable is set.
    sys.exit(0)
else:
    assert solver.available()
    if sys.version_info[:2] < (3, 15):
        assert solver.enabled()
    else:
        assert not solver.enabled()
        sys.exit(0)

# The enabled() check should have triggered MOSEK lazy loading. Trying to get a
# license should confirm the library is working as expected. MOSEK will raise
# an error because the license is not valid.
try:
    solver.AcquireLicense()
    message = None
except Exception as e:
    message = str(e)

# Check `e` for text that indicates the _right_ error.
expected = "Could not acquire MOSEK license"
assert expected in message, message
