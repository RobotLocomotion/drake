import unittest
# import warnings

# from pydrake.common.test_utilities.deprecation import catch_drake_warnings


class TestAtticAll(unittest.TestCase):

    def test_symbols_subset(self):
        """Tests a subset of symbols provided by `drake.attic.all`. At least
        one symbol per submodule should be included.
        """
        import pydrake.attic.all

        # Subset of symbols.
        expected_symbols = (
            "Element",
            "IKResults",
            "RbtInverseDynamics",
            "RgbdCamera",
            "RigidBodyConstraint",
            "RigidBodyPlant",
            "RigidBodyTree",
        )
        # Ensure each symbol is exposed as globals from the above import
        # statement.
        for expected_symbol in expected_symbols:
            self.assertTrue(
                expected_symbol in pydrake.attic.all.__dict__, expected_symbol)
