import subprocess
import textwrap
import unittest

from bazel_tools.tools.python.runfiles import runfiles

from pydrake.common import FindResourceOrThrow
from pydrake.visualization.model_visualizer import ModelVisualizer


class TestModelVisualizerSubprocess(unittest.TestCase):
    """Tests the model_visualizer script as a subprocess."""

    def setUp(self):
        self.manifest = runfiles.Create()
        self.dut = self.manifest.Rlocation(
            "drake/bindings/pydrake/visualization/model_visualizer")

    def model_file(self, model_runpath):
        """Get a model file from a runpath."""
        return self.manifest.Rlocation(model_runpath)

    def test_show(self):
        """Test that show_model doesn't crash."""
        model_runpaths = [
            # Simple SDFormat file.
            "drake/multibody/benchmarks/acrobot/acrobot.sdf",
            # Simple URDF file.
            "drake/multibody/benchmarks/acrobot/acrobot.urdf",
            # Nested SDFormat file.
            "drake/manipulation/util/test/simple_nested_model.sdf",
            # SDFormat world file with multiple models.
            "drake/manipulation/util/test/simple_world_with_two_models.sdf",
        ]
        for model_runpath in model_runpaths:
            print(model_runpath)
            subprocess.check_call([self.dut, self.model_file(model_runpath),
                                   "--loop_once"])

    def test_pyplot(self):
        """Test that pyplot doesn't crash."""
        model_runpath = ("drake/manipulation/models/iiwa_description/sdf/"
                         "iiwa14_no_collision.sdf")
        subprocess.check_call([self.dut, self.model_file(model_runpath),
                               "--loop_once", "--pyplot"])

    def test_set_position(self):
        """Test that the --position option doesn't crash."""
        model_runpaths = [
            # Simple SDFormat file.
            "drake/multibody/benchmarks/acrobot/acrobot.sdf",
            # Simple URDF file.
            "drake/multibody/benchmarks/acrobot/acrobot.urdf",
        ]
        for model_runpath in model_runpaths:
            print(model_runpath)
            subprocess.check_call([self.dut, self.model_file(model_runpath),
                                   "--loop_once", "--position", "0.1", "0.2"])


class TestModelVisualizer(unittest.TestCase):
    """Tests the ModelVisualizer class."""

    SAMPLE_OBJ = textwrap.dedent("""<?xml version="1.0"?>
    <sdf version="1.7">
    <model name="cylinder">
        <pose>0 0 0 0 0 0</pose>
        <link name="cylinder_link">
        <visual name="visual">
            <geometry>
            <cylinder>
                <radius>0.1</radius>
                <length>0.2</length>
            </cylinder>
            </geometry>
        </visual>
        </link>
    </model>
    </sdf>
    """)

    def test_model_in_buffer(self):
        """Tests the ModelVisualizer using a model in a string buffer."""
        dut = ModelVisualizer(visualize_frames=True)
        dut.parser.AddModelsFromString(self.SAMPLE_OBJ, 'sdf')
        dut.Run(position=[1, 0, 0, 0, 0, 0, 0], loop_once=True)

    def test_model_in_file(self):
        """Tests the ModelVisualizer using a model in a file."""
        model_runpaths = [
            # Simple SDFormat file.
            "drake/multibody/benchmarks/acrobot/acrobot.sdf",
            # Simple URDF file.
            "drake/multibody/benchmarks/acrobot/acrobot.urdf",
            # Nested SDFormat file.
            "drake/manipulation/util/test/simple_nested_model.sdf",
            # SDFormat world file with multiple models.
            "drake/manipulation/util/test/simple_world_with_two_models.sdf",
        ]
        for model_runpath in model_runpaths:
            dut = ModelVisualizer(visualize_frames=True)
            dut.AddModels(FindResourceOrThrow(model_runpath))
            dut.Run(loop_once=True)

    def test_methods_and_invariants(self):
        dut = ModelVisualizer()
        self.assertIsNotNone(dut._builder)
        self.assertIsNotNone(dut._plant)
        self.assertIsNotNone(dut._scene_graph)
        self.assertIsNone(dut._diagram)
        self.assertIsNone(dut._sliders)
        self.assertIsNone(dut._context)
        self.assertIsNone(dut._plant_context)

        self.assertIsNotNone(dut._parser)
        # Check that this method can be called twice without error.
        dut.parser.AddModelsFromString(self.SAMPLE_OBJ, 'sdf')

        sample2 = self.SAMPLE_OBJ.replace('name="cylinder"',
                                          'name="cylinder2"')
        dut.parser.AddModelsFromString(sample2, 'sdf')

        dut.Finalize(position=[1, 0, 0, 0, 0, 0, 0] * 2)
        self.assertIsNone(dut._builder)
        self.assertIsNone(dut._scene_graph)
        self.assertIsNone(dut._parser)
        self.assertIsNotNone(dut._diagram)
        self.assertIsNotNone(dut._sliders)
        self.assertIsNotNone(dut._context)
        self.assertIsNotNone(dut._plant_context)

        dut.Run(loop_once=True)
