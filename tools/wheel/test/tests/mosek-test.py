#!/usr/bin/env python

import os

from pydrake.all import MosekSolver

# Configure MOSEK to be enabled at runtime (but with broken license).
os.environ["MOSEKLM_LICENSE_FILE"] = "/does/not/exist.lic"
solver = MosekSolver()
assert solver.enabled()

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
