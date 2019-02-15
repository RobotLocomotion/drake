from __future__ import absolute_import, division, print_function

import sys
import unittest
import warnings


class TestAll(unittest.TestCase):
    # N.B. Synchronize code snippests with `doc/python_bindings.rst`.
    def test_import_warnings(self):
        """Prints if we encounter any warnings (primarily from `pybind11`) when
        importing `pydrake.all`."""
        # Ensure that we haven't yet imported `pydrake.all`.
        self.assertTrue("pydrake.all" not in sys.modules)
        # - While this may be redundant, let's do it for good measure.
        self.assertTrue("pydrake.all" not in sys.modules)
        # Enable *all* warnings, and ensure that we don't trigger them.
        with warnings.catch_warnings(record=True) as w:
            # TODO(eric.cousineau): Figure out a more conservative filter to
            # avoid issues on different machines, but still catch meaningful
            # warnings.
            warnings.simplefilter("always", Warning)
            import pydrake.all
            if w:
                sys.stderr.write("Encountered import warnings:\n{}\n".format(
                    "\n".join(map(str, w)) + "\n"))

    def test_usage_no_all(self):
        from pydrake.common import FindResourceOrThrow
        from pydrake.multibody.parsing import Parser
        from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
        from pydrake.systems.analysis import Simulator
        from pydrake.systems.framework import DiagramBuilder

        builder = DiagramBuilder()
        plant, _ = AddMultibodyPlantSceneGraph(builder)
        Parser(plant).AddModelFromFile(
            FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"))
        plant.Finalize()
        diagram = builder.Build()
        simulator = Simulator(diagram)

    def test_usage_all(self):
        from pydrake.all import (
            AddMultibodyPlantSceneGraph, DiagramBuilder, FindResourceOrThrow,
            Parser, Simulator)

        builder = DiagramBuilder()
        plant, _ = AddMultibodyPlantSceneGraph(builder)
        Parser(plant).AddModelFromFile(
            FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"))
        plant.Finalize()
        diagram = builder.Build()
        simulator = Simulator(diagram)

    def test_usage_all_explicit(self):
        import pydrake.all

        builder = pydrake.systems.framework.DiagramBuilder()
        plant, _ = pydrake.multibody.plant.AddMultibodyPlantSceneGraph(builder)
        pydrake.multibody.parsing.Parser(plant).AddModelFromFile(
            pydrake.common.FindResourceOrThrow(
                "drake/examples/pendulum/Pendulum.urdf"))
        plant.Finalize()
        diagram = builder.Build()
        simulator = pydrake.systems.analysis.Simulator(diagram)

    def test_symbols_subset(self):
        """Tests a subset of symbols provided by `drake.all`. At least one
        symbol per submodule should be included.
        """
        import pydrake.all

        # Subset of symbols.
        expected_symbols = (
            # __init__
            "getDrakePath",
            # attic
            # - solvers
            "RigidBodyConstraint",
            # - systems
            # - - controllers
            "RbtInverseDynamics",
            # - - sensors
            "RgbdCamera",
            # autodiffutils
            "AutoDiffXd",
            # automotive
            "SimpleCar",
            # common
            # - __init__
            "AddResourceSearchPath",
            # - compatibility
            "maybe_patch_numpy_formatters",
            # - containers
            "EqualToDict",
            # - eigen_geometry
            "Isometry3",
            "Quaternion",
            # forwarddiff
            "jacobian",
            "sin",
            "cos",
            # geometry
            "SceneGraph",
            # lcm
            "DrakeLcm",
            # symbolic
            "Variable",
            "Expression",
            # maliput
            # - api
            "RoadGeometry",
            # - dragway
            "create_dragway",
            # manipulation
            # - planner
            "DoDifferentialInverseKinematics",
            # multibody
            # - benchmarks
            "MakeAcrobotPlant",
            # - inverse_kinematics
            "InverseKinematics",
            # - math
            "SpatialVelocity",
            # - multibody_tree
            "BodyNodeIndex",
            "MultibodyTree",
            # - parsing
            "Parser",
            # - parsers
            "PackageMap",
            # - plant
            "MultibodyPlant",
            # - rigid_body_plant
            "RigidBodyPlant",
            # - rigid_body_tree
            "RigidBodyTree",
            # - shapes
            # TODO(eric.cousineau): Avoid collision with `collision.Element`.
            # Import modules, since these names are generic.
            "Element",
            # - tree
            "MultibodyForces",
            # perception
            "PointCloud",
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
            # - controllers
            "InverseDynamics",
            # - lcm
            "PySerializer",
            # - primitives
            "Adder",
            # - rendering
            "PoseVector",
            # - scalar_conversion
            "TemplateSystem",
            # - sensors
            "Image",
        )
        # Ensure each symbol is exposed as globals from the above import
        # statement.
        for expected_symbol in expected_symbols:
            self.assertTrue(
                expected_symbol in pydrake.all.__dict__, expected_symbol)
