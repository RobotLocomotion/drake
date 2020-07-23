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
            warnings.simplefilter("always", Warning)
            # Use `reload` to retrigger the relevant code in
            # `pydrake/__init__.py`.
            importlib.reload(pydrake)

        self.assertEqual(len(caught), 1)
        self.assertEqual(caught[0].category, pydrake._DrakeImportWarning)

        # Show that having `USE_RTLD_GLOBAL_WITH_LIBTORCH` properly set
        # does not trigger the warning.
        torch.USE_RTLD_GLOBAL_WITH_LIBTORCH = False
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always", Warning)
            importlib.reload(pydrake)
        self.assertEqual(len(caught), 0)
