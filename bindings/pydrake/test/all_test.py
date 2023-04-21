import os
import sys
import unittest
import warnings

# When importing pydrake, confirm that it works without any matplotlib
# customization. We don't need the MPLBACKEND=Template override (from
# our tools/bazel.rc file) since importing doesn't open any new windows.
del os.environ['MPLBACKEND']  # noqa

from pydrake.common.test_utilities.deprecation import catch_drake_warnings


class TestAll(unittest.TestCase):
    # N.B. Synchronize code snippets with `doc/_pages/python_bindings.md`.
    def test_000_import_warnings(self):
        """Prints if we encounter any warnings (primarily from `pybind11`) when
        importing `pydrake.all`.

        N.B. This test must run before any others in this class, hence the
        magical name which causes it to sort first.
        """
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
        Parser(plant).AddModels(
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
        Parser(plant).AddModels(
            FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"))
        plant.Finalize()
        diagram = builder.Build()
        simulator = Simulator(diagram)

    def test_usage_all_explicit(self):
        import pydrake.all

        builder = pydrake.systems.framework.DiagramBuilder()
        plant, _ = pydrake.multibody.plant.AddMultibodyPlantSceneGraph(
            builder, 0.0)
        pydrake.multibody.parsing.Parser(plant).AddModels(
            pydrake.common.FindResourceOrThrow(
                "drake/examples/pendulum/Pendulum.urdf"))
        plant.Finalize()
        diagram = builder.Build()
        simulator = pydrake.systems.analysis.Simulator(diagram)

    def test_preferred_ordering(self):
        import pydrake.all
        self.assertIs(pydrake.all.sin, pydrake.math.sin)
        self.assertIs(pydrake.all.Polynomial, pydrake.symbolic.Polynomial)

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
            # - meshcat
            "JointSliders",
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
            # planning
            "RobotDiagram",
            # solvers
            "MathematicalProgram",
            "SnoptSolver",
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
            "AddDefaultVisualization",
            # - _meldis
            "Meldis",
            # -  model_visualizer
            "ModelVisualizer",
            # - _plotting
            "plot_sublevelset_quadratic",
        )
        # Ensure each symbol is exposed as globals from the above import
        # statement.
        for expected_symbol in expected_symbols:
            self.assertTrue(
                expected_symbol in pydrake.all.__dict__, expected_symbol)

    def test_function_only_imports(self):
        """Check for disallowed imports.

        Test that none of the modules which we've decided should only be
        imported by functions appear in sys.modules."""
        import pydrake.all

        # We want to ensure that the following modules are only be imported
        # within a function, not at the module level.
        # E.g., `matplotlib.animation` will freeze `bazel run`.
        function_only_imports = (
            "cv2",
            "matplotlib.animation",
            "matplotlib.pyplot",
            "pydot",
            "scipy",
            )

        bad_imports = list()
        for item in sys.modules.keys():
            for module_name in function_only_imports:
                if item.startswith(module_name):
                    bad_imports.append(item)

        self.assertFalse(
            bad_imports,
            "Function-only import(s) found after importing pydrake.all")
