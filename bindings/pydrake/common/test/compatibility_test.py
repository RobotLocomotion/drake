import pydrake.common.compatibility as mut

import unittest
import warnings

import numpy as np


class TestCompatibility(unittest.TestCase):
    def test_numpy_formatters(self):
        # Call for backwards compatibility.
        with warnings.catch_warnings(record=True) as w:
            mut.maybe_patch_numpy_formatters()
            self.assertEqual(len(w), 1)

    def test_numpy_version(self):
        # This should pass nominally.
        mut.check_required_numpy_version()
        # Mock in an old version.
        with self.assertRaises(RuntimeError):
            mut.check_required_numpy_version(_actual="1.11.0")
