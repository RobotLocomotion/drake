import unittest


class TestBackwardsCompatibility(unittest.TestCase):
    def test_rbtree_shim(self):
        # Test `rbtree` auto-import shim.
        import pydrake
        self.assertTrue(pydrake.rbtree is not None)
        self.assertTrue(pydrake.rbtree.RigidBodyTree is not None)

    def test_rbtree_deprecated(self):
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

    def test_parsers_deprecated(self):
        # Test symbol forwarding.
        from pydrake.parsers import PackageMap
        self.assertTrue(PackageMap is not None)


if __name__ == "__main__":
    unittest.main()
