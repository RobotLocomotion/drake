#!/usr/bin/env python2

"""
Tests that the Drake-installed version of NumPy is used.
"""

import numpy as np
import unittest


class TestNumpyInstall(unittest.TestCase):
    def test_installed_numpy(self):
        # Check the NumPy version.
        # N.B. This should be updated each time the version is bumped.
        expected_version = "1.15.0.dev0+7d247f4"
        self.assertEqual(np.version.version, expected_version)


# Define a main since this is not run via `drake_py_unittest`.
if __name__ == '__main__':
    unittest.main()
