"""
Tests that we can import NumPy, and have the version that we desire.
"""

import numpy as np

import numpy as np
import unittest


class TestNumpyInstall(unittest.TestCase):
    def test_bazel_numpy(self):
        self.assertIn(
            ".runfiles", np.__file__,
            "You must use the Bazel-specified `numpy`! Please ensure your "
            "WORKSPACE and usage of `numpy_py_repository` are well-formed.")
        # N.B. This should be updated each time the version is bumped.
        self.assertEqual("1.15.2", np.version.version)
