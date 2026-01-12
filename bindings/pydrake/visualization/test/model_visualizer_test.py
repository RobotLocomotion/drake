import pydrake.visualization as mut  # ruff: isort: skip
import pydrake.visualization._model_visualizer as mut_private  # ruff: isort: skip  # noqa

import copy
import inspect
import subprocess
import textwrap
import unittest

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import Meshcat
from pydrake.multibody.parsing import PackageMap


class TestModelVisualizerSubprocess(unittest.TestCase):
    """
    Tests the model_visualizer script as a subprocess.

    This also tests the supplied main() method.
    """

    def setUp(self):
        self.dut = FindResourceOrThrow(
            "drake/bindings/pydrake/visualization/model_visualizer"
        )

    def model_file(self, *, model_url):
        """Get a model file from a URL."""
        return PackageMap().ResolveUrl(model_url)

    def test_model_visualizer(self):
        """Test that model_visualizer doesn't crash."""
        model_urls = [
            # Simple SDFormat file.
            "package://drake/multibody/benchmarks/acrobot/acrobot.sdf",
            # Simple URDF file.
            "package://drake/multibody/benchmarks/acrobot/acrobot.urdf",
            # Nested SDFormat file.
            "package://drake/manipulation/util/test/simple_nested_model.sdf",
            # SDFormat world file with multiple models.
            "package://drake/manipulation/util/test/"
            + "simple_world_with_two_models.sdf",
        ]
        for i, model_url in enumerate(model_urls):
            with self.subTest(model_url=model_url):
                filename = self.model_file(model_url=model_url)
                args = [self.dut, filename, "--loop_once"]
                # Smoke test coverage of optional command line options.
                if i % 2 == 1:
                    args.append("--compliance_type=compliant")
                subprocess.check_call(args)

    def test_model_with_invalid_dynamics(self):
        """
        Test on a model with invalid dynamics.
        The visualizer script will disable visualization of contact forces.
        """

        # Model containing a free body with zero inertias.
        # Obviously, physics cannot be computed, and thus visualization of
        # contact forces is turned off.
        result = subprocess.run(
            [
                self.dut,
                "bindings/pydrake/visualization/test/massless_robot.urdf",
                "--loop_once",
            ],
            stderr=subprocess.PIPE,
            text=True,
        )

        # If the model is handled as expected, the visualizer script prints a
        # WARNING message.
        self.assertRegex(result.stderr, "WARNING.*Contact results cannot")

    def test_package_url(self):
        """Test that a package URL works."""
        subprocess.check_call(
            [
                self.dut,
                "package://drake/multibody/benchmarks/acrobot/acrobot.sdf",
                "--loop_once",
            ]
        )

    def test_pyplot(self):
        """Test that pyplot doesn't crash."""
        model_url = (
            "package://drake_models/iiwa_description/sdf/"
            + "iiwa14_no_collision.sdf"
        )
        subprocess.check_call(
            [
                self.dut,
                self.model_file(model_url=model_url),
                "--loop_once",
                "--pyplot",
            ]
        )

    def test_set_position(self):
        """Test that the --position option doesn't crash."""
        model_urls = [
            # Simple SDFormat file.
            "package://drake/multibody/benchmarks/acrobot/acrobot.sdf",
            # Simple URDF file.
            "package://drake/multibody/benchmarks/acrobot/acrobot.urdf",
        ]
        for model_url in model_urls:
            print(model_url)
            filename = self.model_file(model_url=model_url)
            subprocess.check_call(
                [self.dut, filename, "--loop_once", "--position", "0.1", "0.2"]
            )


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
        model_urls = [
            # Simple SDFormat file.
            "package://drake/multibody/benchmarks/acrobot/acrobot.sdf",
            # Simple URDF file.
            "package://drake/multibody/benchmarks/acrobot/acrobot.urdf",
            # Nested SDFormat file.
            "package://drake/manipulation/util/test/simple_nested_model.sdf",
            # SDFormat world file with multiple models.
            "package://drake/manipulation/util/test/"
            + "simple_world_with_two_models.sdf",
            # glTF file.
            "package://drake_models/veggies/assets/"
            + "yellow_bell_pepper_no_stem_low.gltf",
        ]
        for i, model_url in enumerate(model_urls):
            with self.subTest(model=model_url):
                filename = PackageMap().ResolveUrl(model_url)
                dut = mut.ModelVisualizer(visualize_frames=True)
                if i % 2 == 0:
                    # Conditionally Ping the parser() here, so that we cover
                    # both branching paths within AddModels().
                    dut.parser()
                dut.AddModels(filename=filename)
                dut.Run(loop_once=True)

    def test_model_from_url(self):
        url = "package://drake/multibody/benchmarks/acrobot/acrobot.sdf"
        dut = mut.ModelVisualizer()
        dut.AddModels(url=url)
        dut.Run(loop_once=True)

    def test_model_from_gltf_url(self):
        url = (
            "package://drake_models/veggies/assets/"
            + "yellow_bell_pepper_no_stem_low.gltf"
        )
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

        sample2 = self.SAMPLE_OBJ.replace('name="cylinder"', 'name="cylinder2"')
        dut.parser().AddModelsFromString(sample2, "sdf")

        positions = [1, 0, 0, 0, 0, 0, 0] * 2  # Model is just doubled.
        dut.Finalize(position=positions)
        dut.Run(position=positions, loop_once=True)

    def test_hydroelastic_contact(self):
        """
        When hydroelastic contact is configured, the right kind of contacts
        appear in Meshcat.
        """
        meshcat = Meshcat()
        dut = mut.ModelVisualizer(compliance_type="compliant", meshcat=meshcat)
        for model in [
            "planning/test_utilities/collision_ground_plane.sdf",
            "manipulation/util/test/simple_nested_model.sdf",
        ]:
            dut.AddModels(url=f"package://drake/{model}")
        dut.Run(loop_once=True)
        self.assertTrue(
            meshcat._GetPackedProperty(
                path="+".join(
                    [
                        "contact_forces/hydroelastic/ground_plane_box",
                        "link/force_C_W",
                    ]
                ),
                property="visible",
            )
        )

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

    def test_visualize_all_frames(self):
        """Confirm that *all* frames get added."""
        dut = mut.ModelVisualizer(visualize_frames=True)
        dut.parser().AddModels(
            file_type=".urdf",
            file_contents="""
<?xml version="1.0"?>
<robot name="test_model">
    <link name="box">
        <inertial>
        <mass value="1"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
        <visual>
        <geometry>
            <box size=".1 .2 .3"/>
        </geometry>
        </visual>
    </link>
    <frame name="offset_frame" link="box" xyz="0.25 0 0" rpy="0 0 0"/>
</robot>
""",
        )
        dut.Run(loop_once=True)

        meshcat = dut.meshcat()
        self.assertTrue(meshcat.HasPath("/drake/illustration/test_model"))
        # Body frame.
        self.assertTrue(
            meshcat.HasPath(
                "/drake/illustration/test_model/box/_frames/box(2)/x-axis"
            )
        )
        self.assertTrue(
            meshcat.HasPath(
                "/drake/illustration/test_model/box/_frames/box(2)/y-axis"
            )
        )
        self.assertTrue(
            meshcat.HasPath(
                "/drake/illustration/test_model/box/_frames/box(2)/z-axis"
            )
        )
        # Offset frame.
        self.assertTrue(
            meshcat.HasPath(
                "/drake/illustration/test_model/box"
                "/_frames/offset_frame(2)/x-axis"
            )
        )
        self.assertTrue(
            meshcat.HasPath(
                "/drake/illustration/test_model/box"
                "/_frames/offset_frame(2)/y-axis"
            )
        )
        self.assertTrue(
            meshcat.HasPath(
                "/drake/illustration/test_model/box"
                "/_frames/offset_frame(2)/z-axis"
            )
        )

    def test_visualize_all_frames_nested_models(self):
        """When parsing SDFormat with nested models, we can get multiple
        __model__ frames associated with the same body. It should still
        process the frames and not complain about repeated geometry names.
        """
        sdf_file = FindResourceOrThrow(
            "drake/multibody/parsing/test/sdf_parser_test/"
            "model_with_directly_nested_models.sdf"
        )

        dut = mut.ModelVisualizer(visualize_frames=True)
        dut.AddModels(filename=sdf_file)
        dut.Run(loop_once=True)
        # Note: reaching here without throwing is enough.
