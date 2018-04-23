"""
Tests that we can import NumPy, and have the version that we desire.
"""

import numpy as np

assert ".runfiles" in np.__file__, (
    "You must use the Bazel-specified `numpy`! Please ensure your WORKSPACE "
    "and usage of `numpy_py_repository` are well-formed.")
