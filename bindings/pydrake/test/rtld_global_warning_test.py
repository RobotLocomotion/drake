import importlib
import sys
import unittest
import warnings

import pydrake


class TestRtldGlobalWarning(unittest.TestCase):
    def test_mock_torch(self):
        # Import the mock module.
        import torch

        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("once", Warning)
            # Use `reload` to retrigger the relevant code in
            # `pydrake/__init__.py`.
            importlib.reload(pydrake)

        self.assertEqual(len(caught), 1)
        self.assertEqual(caught[0].category, pydrake._DrakeImportWarning)
