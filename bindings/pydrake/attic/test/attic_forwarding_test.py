"""
Tests existence of all relevant attic forwarding symbols.
"""

from contextlib import contextmanager
import unittest
import warnings

from pydrake.common.deprecation import DrakeDeprecationWarning


class TestAtticForwarding(unittest.TestCase):
    @contextmanager
    def expect_deprecation(self):
        with warnings.catch_warnings():
            with self.assertRaises(DrakeDeprecationWarning):
                warnings.simplefilter("error", DrakeDeprecationWarning)
                yield

    def test_multibody_modules(self):
        with self.expect_deprecation():
            import pydrake.multibody.collision
        with self.expect_deprecation():
            import pydrake.multibody.joints
        with self.expect_deprecation():
            import pydrake.multibody.parsers
        with self.expect_deprecation():
            import pydrake.multibody.rigid_body_plant
        with self.expect_deprecation():
            import pydrake.multibody.rigid_body
        with self.expect_deprecation():
            import pydrake.multibody.rigid_body_tree
        with self.expect_deprecation():
            import pydrake.multibody.shapes

    def test_solvers_modules(self):
        with self.expect_deprecation():
            import pydrake.solvers.ik
