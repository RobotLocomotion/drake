import unittest
import sys
import warnings

from pydrake.util.deprecation import DrakeDeprecationWarning


class TestBackwardsCompatibility(unittest.TestCase):
    def tearDown(self):
        import pydrake
        # N.B. `reload(pydrake)` is more complicated because it is a
        # `ModuleShim` instance.
        if "pydrake.rbtree" in sys.modules:
            del sys.modules["pydrake.rbtree"]
            delattr(pydrake, "rbtree")

    def test_rbtree_shim(self):
        # Test `rbtree` auto-import shim.
        import pydrake
        with warnings.catch_warnings(record=True) as w:
            # See notes in `deprecation_test`.
            warnings.simplefilter("default", DrakeDeprecationWarning)
            self.assertTrue(pydrake.rbtree is not None)
            self.assertTrue(pydrake.rbtree.RigidBodyTree is not None)
            self.assertEqual(len(w), 1)

    def test_rbtree_import(self):
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("default", DrakeDeprecationWarning)
            import pydrake.rbtree
            self.assertEqual(len(w), 1)

    def test_rbtree_deprecated(self):
        with warnings.catch_warnings(record=True) as w:
            # See notes in `deprecation_test`.
            warnings.simplefilter("default", DrakeDeprecationWarning)
            # Test symbol forwarding.
            from pydrake.rbtree import (
                # Subset of actual symbols.
                RigidBodyTree,
                FloatingBaseType,
                # All aliases.
                PackageMap,
                kFixed,
                kRollPitchYaw,
                kQuaternion,
            )
            self.assertTrue(RigidBodyTree is not None)
            self.assertEqual(len(w), 1)

    def test_parsers_deprecated(self):
        # Test symbol forwarding.
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("default", DrakeDeprecationWarning)
            from pydrake.parsers import PackageMap
            self.assertTrue(PackageMap is not None)
            self.assertEqual(len(w), 1)
