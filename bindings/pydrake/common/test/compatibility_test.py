import pydrake.common.compatibility as mut

import numpy as np
import unittest

numpy_formatters = mut._patches["numpy_formatters"]

# TODO(eric.cousineau): Try to reproduce NumPy formatter patch with mock types.
# Even with creating a `MockSymbol` type which returns a `MockFormula` which
# raises an error on `__nonzero__`, I could not get the error relevant to #8729
# to trigger via `np.maximum.reduce` (but could trigger the error via
# `np.max`).


class TestCompatibility(unittest.TestCase):
    @unittest.skipIf(not numpy_formatters["required"], "Patch not required")
    def test_numpy_formatters(self):
        # Ensure that the expected types have been patched with the `Deferred`
        # type.
        expect_deferred = [
            'IntegerFormat',
            'FloatFormat',
        ]
        # Before patching.
        self.assertFalse(numpy_formatters["applied"])
        module = np.core.arrayprint
        for name in expect_deferred:
            self.assertNotIn("Deferred", str(getattr(module, name)))
        # Apply patch.
        mut.maybe_patch_numpy_formatters()
        self.assertTrue(numpy_formatters["applied"])
        for name in expect_deferred:
            self.assertIn("Deferred", str(getattr(module, name)))
