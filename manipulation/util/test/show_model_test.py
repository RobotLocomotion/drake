import subprocess
import textwrap
import unittest

from bazel_tools.tools.python.runfiles import runfiles

from drake.manipulation.util.show_model import ModelVisualizer


class TestShowModel(unittest.TestCase):
    """Tests the show_model script as a subprocess."""

    def setUp(self):
        self.manifest = runfiles.Create()
        self.dut = self.manifest.Rlocation(
            "drake/manipulation/util/show_model")

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
