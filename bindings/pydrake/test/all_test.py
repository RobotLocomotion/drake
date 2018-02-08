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

    def test_symbols_subset(self):
        """Tests a subset of symbols provided by `drake.all`. At least one
        symbol per submodule should be included.
        """
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
            # - parsers
            "PackageMap",
            # - rigid_body_plant
            "RigidBodyPlant",
            # - rigid_body_tree
            "RigidBodyTree",
            # - shapes
            # TODO(eric.cousineau): Avoid collision with `collision.Element`.
            # Import modules, since these names are generic.
            "Element",
            # solvers
            # - gurobi
            "GurobiSolver",
            # - ik
            "IKResults",
            # - ipopt
            "IpoptSolver",
            # - mathematicalprogram
            "MathematicalProgram",
            # - mosek
            "MosekSolver",
            # systems
            # - framework
            "BasicVector",
            "LeafSystem",
            # - analysis
            "Simulator",
            # - primitives
            "Adder",
            # - rendering
            "PoseVector",
            # - sensors
            "Image",
            # util
            "Isometry3",
            "Quaternion",
        )
        # Ensure each symbol is exposed as globals from the above import
        # statement.
        for expected_symbol in expected_symbols:
            self.assertTrue(
                expected_symbol in pydrake.all.__dict__, expected_symbol)


if __name__ == '__main__':
    unittest.main()
