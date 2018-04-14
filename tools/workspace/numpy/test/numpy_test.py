"""
Tests that we can import NumPy, and have the version that we desire.
Note that we do not *require* the Bazel version, as sometimes it is convenient
to use development versions.
"""

import numpy as np
import sys

if ".runfiles" in np.__file__:
    print("Using Bazel-specified `numpy`.")
else:
    sys.stderr.write(
        "NOT using a Bazel-specified `numpy` binary! Use at your own risk!\n")
