import subprocess
import textwrap
import unittest

from pydrake.common import FindResourceOrThrow
import pydrake.visualization as mut


class TestModelVisualizerSubprocess(unittest.TestCase):
    """
    Tests the model_visualizer script as a subprocess.

    This also tests the supplied main() method.
    """

    def setUp(self):
        self.dut = FindResourceOrThrow(
            "drake/bindings/pydrake/visualization/model_visualizer")

    def model_file(self, model_runpath):
        """Get a model file from a runpath."""
        return FindResourceOrThrow(model_runpath)

    def test_model_visualizer(self):
        """Test that model_visualizer doesn't crash."""
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

    def test_package_url(self):
        """Test that a package URL works."""
        subprocess.check_call([
            self.dut,
            "package://drake/multibody/benchmarks/acrobot/acrobot.sdf",
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
        """
        Tests the ModelVisualizer using a model in a string buffer.

        Also tests getters and re-using a Meshcat instance.
        """
        visualize_frames = True
        triad_length = 0.4
        triad_radius = 0.02
        triad_opacity = 0.5
        publish_contacts = False
        browser_new = False
        pyplot = True

        dut = mut.ModelVisualizer(
            visualize_frames=visualize_frames, triad_length=triad_length,
            triad_radius=triad_radius, triad_opacity=triad_opacity,
            publish_contacts=publish_contacts, browser_new=browser_new,
            pyplot=pyplot)

        self.assertEqual(dut.visualize_frames, visualize_frames)
        self.assertEqual(dut.triad_length, triad_length)
        self.assertEqual(dut.triad_radius, triad_radius)
        self.assertEqual(dut.triad_opacity, triad_opacity)
        self.assertEqual(dut.publish_contacts, publish_contacts)
        self.assertEqual(dut.browser_new, browser_new)
        self.assertEqual(dut.pyplot, pyplot)

        dut.parser.AddModelsFromString(self.SAMPLE_OBJ, 'sdf')
        dut.Run(position=[1, 0, 0, 0, 0, 0, 0], loop_once=True)

        dut2 = mut.ModelVisualizer(meshcat=dut.meshcat)
        dut2.parser.AddModelsFromString(self.SAMPLE_OBJ, 'sdf')
        dut.Run(loop_once=True)

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
            dut = mut.ModelVisualizer(visualize_frames=True)
            dut.AddModels(FindResourceOrThrow(model_runpath))
            dut.Run(loop_once=True)

    def test_methods_and_multiple_models(self):
        """
        Tests main class methods individually as well as adding models twice.
        Also tests passing `position` to both Finalize and Run.
        """
        dut = mut.ModelVisualizer()

        # Check that models can be added multiple times without error.
        dut.parser.AddModelsFromString(self.SAMPLE_OBJ, 'sdf')

        sample2 = self.SAMPLE_OBJ.replace('name="cylinder"',
                                          'name="cylinder2"')
        dut.parser.AddModelsFromString(sample2, 'sdf')

        positions = [1, 0, 0, 0, 0, 0, 0] * 2  # Model is just doubled.
        dut.Finalize(position=positions)
        dut.Run(position=positions, loop_once=True)
