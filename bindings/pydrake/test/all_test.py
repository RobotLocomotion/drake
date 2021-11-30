import sys
import unittest
import warnings

from pydrake.common.test_utilities.deprecation import catch_drake_warnings


class TestAll(unittest.TestCase):
    # N.B. Synchronize code snippests with `doc/python_bindings.rst`.
    def test_import_warnings(self):
        """Prints if we encounter any warnings (primarily from `pybind11`) when
        importing `pydrake.all`."""
        # Ensure that we haven't yet imported `pydrake.all`.
        self.assertTrue("pydrake.all" not in sys.modules)
        # - While this may be redundant, let's do it for good measure.
        self.assertTrue("pydrake.all" not in sys.modules)
        # Catch all warnings using their normal specification.
        with warnings.catch_warnings(record=True) as w:
            warnings.filterwarnings(
                "ignore", message="Matplotlib is building the font cache",
                category=UserWarning)
            warnings.filterwarnings(
                "ignore", message=".* from 'collections.abc' is deprecated",
                category=DeprecationWarning)
            import pydrake.all
            self.assertEqual(len(w), 0, [x.message for x in w])

    def test_usage_no_all(self):
        from pydrake.common import FindResourceOrThrow
        from pydrake.multibody.parsing import Parser
        from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
        from pydrake.systems.analysis import Simulator
        from pydrake.systems.framework import DiagramBuilder

        builder = DiagramBuilder()
        plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
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
        plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
        Parser(plant).AddModelFromFile(
            FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"))
        plant.Finalize()
        diagram = builder.Build()
        simulator = Simulator(diagram)

    def test_usage_all_explicit(self):
        import pydrake.all

        builder = pydrake.systems.framework.DiagramBuilder()
        plant, _ = pydrake.multibody.plant.AddMultibodyPlantSceneGraph(
            builder, 0.0)
        pydrake.multibody.parsing.Parser(plant).AddModelFromFile(
            pydrake.common.FindResourceOrThrow(
                "drake/examples/pendulum/Pendulum.urdf"))
        plant.Finalize()
        diagram = builder.Build()
        simulator = pydrake.systems.analysis.Simulator(diagram)

    def test_math_overloads_shadowing(self):
        """Tests that we end up with the more capable pydrake.math functions in
        the all module, not the more limited pydrake.symbolic functions.
        """
        from pydrake.all import AutoDiffXd, Expression, Variable, sin
        self.assertIsInstance(sin(1), float)
        self.assertIsInstance(sin(AutoDiffXd(1, [1, 0])), AutoDiffXd)
        self.assertIsInstance(sin(Expression(Variable('e'))), Expression)
        self.assertIsInstance(sin(Variable('x')), Expression)

    def test_symbols_subset(self):
        """Tests a subset of symbols provided by `drake.all`. At least one
        symbol per submodule should be included.
        """
        import pydrake.all

        # Subset of symbols.
        expected_symbols = (
            # __init__
            "getDrakePath",
            # autodiffutils
            "AutoDiffXd",
            # common
            # - __init__
            "RandomDistribution",
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
            # yaml
            "yaml_load_data",
            # schema
            "ToDistributionVector",
            # - render
            "ColorRenderCamera",
            # lcm
            "DrakeLcm",
            # symbolic
            "Variable",
            "Expression",
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
            # - parsing
            "Parser",
            # - parsers
            "PackageMap",
            # - plant
            "MultibodyPlant",
            # - tree
            "MultibodyForces",
            # perception
            "PointCloud",
            # solvers
            # - mixed_integer_optimization_util
            "AddLogarithmicSos2Constraint",
            # - gurobi
            "GurobiSolver",
            # - sdpa_free_format
            "GenerateSDPA",
            # - ipopt
            "IpoptSolver",
            # - branch_and_bound
            "MixedIntegerBranchAndBound",
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
            # - scalar_conversion
            "TemplateSystem",
            # - sensors
            "Image",
            # visualization
            # - plotting
            "plot_sublevelset_quadratic",
        )
        # Ensure each symbol is exposed as globals from the above import
        # statement.
        for expected_symbol in expected_symbols:
            self.assertTrue(
                expected_symbol in pydrake.all.__dict__, expected_symbol)
