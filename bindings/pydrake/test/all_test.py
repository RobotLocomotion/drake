from __future__ import absolute_import, division, print_function

import unittest
import pydrake.common

from pydrake.all import *


class TestAll(unittest.TestCase):
    def test_symbols(self):
        # Subset of symbols.
        expected_symbols = (
            # autodiffutils
            "AutoDiffXd",
            # common
            "AddResourceSearchPath",
            # parsers
            "PackageMap",
            # rbtree
            "RigidBodyTree",
            # symbolic
            "Variable",
            "Expression",
            # multibody
            "RigidBodyPlant",
            # solvers
            "MathematicalProgram",
            "LinearConstraint",
            # systems
            "BasicVector",
            "LeafSystem",
            "Simulator",
        )
        # Ensure each symbol is exposed as globals from the above import
        # statement.
        for expected_symbol in expected_symbols:
            assert expected_symbol in globals()


if __name__ == '__main__':
    unittest.main()
