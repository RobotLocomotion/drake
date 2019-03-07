import pydrake.common.compatibility as mut

from functools import partial
from threading import Thread
import unittest

import numpy as np

from pydrake.common.compatibility_test_util import invoke_callback

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

    def test_gil_callback(self):
        # Ensure that GIL behavior is correct, and does not cause threads to
        # hang when pybind11 invokes Python callbacks from C++ (#10471).
        called = [False]  # Store as list for easy non-local mutation.

        def callback():
            called[0] = True

        thread = Thread(target=partial(invoke_callback, callback))
        thread.start()
        thread.join()
        self.assertTrue(called[0])
