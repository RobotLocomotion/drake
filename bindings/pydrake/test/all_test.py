from __future__ import absolute_import, division, print_function

import sys
import unittest
import warnings


class TestAll(unittest.TestCase):
    # N.B. Synchronize code snippests with `doc/python_bindings.rst`.
    def test_no_import_warnings(self):
        """Ensures that we have no warnings (primarily from `pybind11`) when
        importing `pydrake.all`."""
        # Ensure that we haven't imported anything from `pydrake`.
        self.assertTrue("pydrake" not in sys.modules)
        # - While this may be redundant, let's do it for good measure.
        self.assertTrue("pydrake.all" not in sys.modules)
        # Enable *all* warnings, and ensure that we don't trigger them.
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always", Warning)
            import pydrake.all
            self.assertEquals(len(w), 0)

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
            # automotive
            "SimpleCar",
            # common
            "AddResourceSearchPath",
            # forwarddiff
            "jacobian",
            "sin",
            "cos",
            # symbolic
            "Variable",
            "Expression",
            # maliput
            # - api
            "RoadGeometry",
            # - dragway
            "create_dragway",
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
            # - multibody_tree
            "SpatialVelocity",
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
