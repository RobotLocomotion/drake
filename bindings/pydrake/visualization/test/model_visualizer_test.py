import copy
import inspect
import subprocess
import textwrap
import time
import unittest

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import Meshcat
import pydrake.visualization as mut
import pydrake.visualization._model_visualizer as mut_private


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
    """
    Tests the ModelVisualizer class.

    Note that camera tests are split into the model_visualizer_camera_test, and
    reload tests are split into the model_visualizer_reload_test.
    """

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

    def setUp(self):
        mut_private._webbrowser_open = self._mock_webbrowser_open
        # When we want to allow webbrowser.open, this will be a list() that
        # captures the kwargs it was called with.
        self._webbrowser_opened = None

    def _mock_webbrowser_open(self, *args, **kwargs):
        self.assertEqual(args, tuple())
        if self._webbrowser_opened is None:
            self.fail("Unexpected webbrowser.open")
        self._webbrowser_opened.append(copy.deepcopy(kwargs))

    def test_model_from_string(self):
        """Visualizes a model from a string buffer."""
        dut = mut.ModelVisualizer()
        dut.parser().AddModelsFromString(self.SAMPLE_OBJ, "sdf")
        self.assertIsNotNone(dut.meshcat())
        dut.Run(position=[1, 0, 0, 0, 0, 0, 0], loop_once=True)

    def test_provided_meshcat(self):
        """Visualizes using a user-provided meshcat."""
        meshcat = Meshcat()
        dut = mut.ModelVisualizer(meshcat=meshcat)
        self.assertEqual(dut.meshcat(), meshcat)
        dut.parser().AddModelsFromString(self.SAMPLE_OBJ, "sdf")
        dut.Run(loop_once=True)

    def test_pyplot(self):
        """Enables pyplot and ensures nothing crashes."""
        # This acute test only makes sense when pyplot is disabled by default.
        defaults = mut.ModelVisualizer._get_constructor_defaults()
        assert defaults["pyplot"] is False
        dut = mut.ModelVisualizer(pyplot=True)
        dut.parser().AddModelsFromString(self.SAMPLE_OBJ, "sdf")
        dut.Run(loop_once=True)

    def test_model_from_file(self):
        """
        Visualizes models from files, one at a time.
        Enables frame visualization for additional code coverage.
        """
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
        for i, model_runpath in enumerate(model_runpaths):
            with self.subTest(model=model_runpath):
                dut = mut.ModelVisualizer(visualize_frames=True)
                if i % 2 == 0:
                    # Conditionally Ping the parser() here, so that we cover
                    # both branching paths within AddModels().
                    dut.parser()
                dut.AddModels(FindResourceOrThrow(model_runpath))
                dut.Run(loop_once=True)

    def test_model_from_url(self):
        url = "package://drake/multibody/benchmarks/acrobot/acrobot.sdf"
        dut = mut.ModelVisualizer()
        dut.AddModels(url=url)
        dut.Run(loop_once=True)

    def test_add_model_args_error(self):
        filename = "drake/multibody/benchmarks/acrobot/acrobot.sdf"
        url = f"package://{filename}"
        dut = mut.ModelVisualizer()
        with self.assertRaisesRegex(ValueError, "either filename.*url"):
            dut.AddModels(filename, url=url)

    def test_methods_and_multiple_models(self):
        """
        Tests main class methods individually as well as adding models twice.
        Also tests passing `position` to both Finalize and Run.
        """
        dut = mut.ModelVisualizer()

        # Check that models can be added multiple times without error.
        dut.parser().AddModelsFromString(self.SAMPLE_OBJ, "sdf")

        sample2 = self.SAMPLE_OBJ.replace('name="cylinder"',
                                          'name="cylinder2"')
        dut.parser().AddModelsFromString(sample2, "sdf")

        positions = [1, 0, 0, 0, 0, 0, 0] * 2  # Model is just doubled.
        dut.Finalize(position=positions)
        dut.Run(position=positions, loop_once=True)

    def test_precondition_messages(self):
        dut = mut.ModelVisualizer()
        dut.Finalize()
        with self.assertRaisesRegex(ValueError, "already been"):
            dut.package_map()
        with self.assertRaisesRegex(ValueError, "already been"):
            dut.parser()
        with self.assertRaisesRegex(ValueError, "already been"):
            dut.AddModels("ignored.urdf")

    def test_traffic_cone(self):
        """
        Checks that the traffic cone helpers don't crash.
        """
        meshcat = Meshcat()
        dut = mut.ModelVisualizer(meshcat=meshcat)
        path = "/PARSE_ERROR"
        # The add & removes functions must work even when called too many, too
        # few, or an unmatched number of times. It's not practical to have the
        # calling code keep track of how often things are called, so the code
        # must be robust to any ordering.
        for _ in range(2):
            dut._add_traffic_cone()
        self.assertTrue(dut._meshcat.HasPath(path))
        for _ in range(3):
            dut._remove_traffic_cone()
        self.assertFalse(dut._meshcat.HasPath(path))

    def test_webbrowser(self):
        """
        Checks that the webbrowser launch command is properly invoked.
        """
        # If the browser is opened in this stanza, the test will fail.
        dut = mut.ModelVisualizer(browser_new=True)
        dut.parser().AddModelsFromString(self.SAMPLE_OBJ, "sdf")
        dut.Finalize()

        # Now we allow (mocked) webbrowser.open.
        self._webbrowser_opened = list()
        dut.Run(loop_once=True)

        # Check that was called exactly once, with correct kwargs.
        self.assertEqual(len(self._webbrowser_opened), 1)
        kwargs = self._webbrowser_opened[0]
        self.assertEqual(kwargs["new"], True)
        self.assertIn("localhost", kwargs["url"])
        self.assertEqual(len(kwargs), 2)

    def test_triad_defaults(self):
        # Cross-check the default triad parameters.
        expected = inspect.signature(mut.AddFrameTriadIllustration).parameters
        actual = mut.ModelVisualizer._get_constructor_defaults()
        for name in ("length", "radius", "opacity"):
            self.assertEqual(actual[f"triad_{name}"], expected[name].default)
