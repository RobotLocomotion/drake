from __future__ import absolute_import, division, print_function

import unittest


class TestAll(unittest.TestCase):
    # N.B. Synchronize code snippests with `doc/python_bindings.rst`.

    def test_usage_no_all(self):
        from pydrake.common import FindResourceOrThrow
        from pydrake.multibody.rigid_body_plant import RigidBodyPlant
        from pydrake.multibody.rigid_body_tree import RigidBodyTree
        from pydrake.systems.analysis import Simulator

        tree = RigidBodyTree(
            FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"))
        simulator = Simulator(RigidBodyPlant(tree))

    def test_usage_all(self):
        from pydrake.all import (
            FindResourceOrThrow, RigidBodyPlant, RigidBodyTree, Simulator)

        tree = RigidBodyTree(
            FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"))
        simulator = Simulator(RigidBodyPlant(tree))

    def test_usage_all_explicit(self):
        import pydrake.all

        tree = pydrake.multibody.rigid_body_tree.RigidBodyTree(
            pydrake.common.FindResourceOrThrow(
                "drake/examples/pendulum/Pendulum.urdf"))
        simulator = pydrake.systems.analysis.Simulator(
            pydrake.multibody.rigid_body_plant.RigidBodyPlant(tree))

    def test_symbols(self):
        import pydrake.all

        # Subset of symbols.
        expected_symbols = (
            # autodiffutils
            "AutoDiffXd",
            # common
            "AddResourceSearchPath",
            # forwarddiff
            "jacobian",
            "sin",
            "cos",
            # symbolic
            "Variable",
            "Expression",
            # multibody
            "PackageMap",
            "RigidBodyPlant",
            "RigidBodyTree",
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
            self.assertTrue(expected_symbol in pydrake.all.__dict__)


if __name__ == '__main__':
    unittest.main()
