import pydrake.visualization as mut  # ruff: isort: skip

from contextlib import redirect_stderr, redirect_stdout
import importlib
import io
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np
import umsgpack

import pydrake.common.test_utilities.numpy_compare as numpy_compare
import pydrake.geometry as mut_geometry
from pydrake.geometry import (
    Meshcat,
    RenderEngineGlParams,
    RenderEngineVtkParams,
)
from pydrake.math import (
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
)
from pydrake.systems.sensors import ImageRgba8U
import pydrake.visualization._model_visualizer as mut_private
import pydrake.visualization.model_visualizer as model_visualizer_cli


class TestModelVisualizerCamera(unittest.TestCase):
    """
    Tests the camera feature of the ModelVisualizer class.

    The camera testing is carved into a separate file vs model_visualizer_test
    because our camera code tends to be flaky when run using the emulated video
    driver on our continuous integration builds.
    """

    def test_camera(self):
        """
        Checks that the rgbd sensor code tracks the browser's camera pose (and
        doesn't crash).
        """
        # Create a meshcat instance as if a browser had connected and sent its
        # camera pose.
        meshcat = Meshcat()
        meshcat._InjectWebsocketMessage(
            message=umsgpack.packb(
                {
                    "type": "camera_pose",
                    "camera_pose": [
                        1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 0,
                        1, 2, 3, 1,
                    ],
                    "is_perspective": True,
                }
            )
        )  # fmt: skip
        # Transform y-up to z-up, and from facing in the +z direction to the -z
        # direction (with concomitant flip of the y-axis).
        X_WB_expected = RigidTransform(
            R=RotationMatrix(RollPitchYaw(np.pi / 2, np.pi, np.pi)),
            p=[1.0, -3.0, 2.0],
        )

        # N.B. We don't need perception geometry in the scene -- we'll rely on
        # the RgbdSensor unit tests to check that cameras work as advertised.
        # Our only goal here is to achieve statement-level code coverage of the
        # ModelVisualizer code when show_rgbd_sensor is enabled.
        model = """<?xml version="1.0"?>
          <sdf version="1.9">
            <model name="sample">
              <link name="base"/>
            </model>
          </sdf>
        """
        dut = mut.ModelVisualizer(meshcat=meshcat, show_rgbd_sensor="vtk")
        dut.parser().AddModelsFromString(model, "sdf")
        dut.Run(loop_once=True)

        # Confirm that the pose got updated properly. Updated pose is a proxy
        # for the full behavior in dut._render_if_necessary().
        camera_frame = dut._diagram.plant().GetFrameByName(
            "$rgbd_sensor_offset"
        )
        X_WB = camera_frame.GetPoseInParentFrame(
            dut._diagram.plant().GetMyContextFromRoot(dut._context)
        )

        numpy_compare.assert_allclose(
            X_WB.GetAsMatrix34(), X_WB_expected.GetAsMatrix34(), atol=1e-15
        )

        camera_sensor = dut._diagram.GetSubsystemByName("rgbd_sensor_preview")
        color_camera = camera_sensor.default_color_render_camera()
        self.assertEqual(color_camera.core().clipping().far(), 500.0)
        depth_camera = camera_sensor.default_depth_render_camera()
        self.assertEqual(depth_camera.depth_range().max_depth(), 3.0)
        preview_image = dut._diagram.GetOutputPort("preview_image").Eval(
            dut._context
        )
        self.assertIsInstance(preview_image, ImageRgba8U)

    def test_show_rgbd_sensor_configuration(self):
        """Checks that the preview camera gets the selected renderer."""

        for value, expected in [
            (None, None),
            (False, None),
            (True, "vtk"),
            ("vtk", "vtk"),
        ]:
            with self.subTest(value=value):
                dut = mut.ModelVisualizer(show_rgbd_sensor=value)
                self.assertEqual(dut._show_rgbd_sensor, expected)

        # Note: we're specifically requesting show_rgbd_sensor, because
        # _make_rgbd_sensor_config() is only invoked for that setting.
        vtk_config = mut.ModelVisualizer(
            show_rgbd_sensor="vtk"
        )._make_rgbd_sensor_config()
        self.assertIsInstance(vtk_config.renderer_class, RenderEngineVtkParams)

        # Note: we're not instantiating the render engine, just configuring
        # one. So, we can safely "lie" about the availability.
        with patch.object(
            mut.ModelVisualizer,
            "_supported_rgbd_renderers",
            ("vtk", "gl"),
        ):
            gl_config = mut.ModelVisualizer(
                show_rgbd_sensor="gl"
            )._make_rgbd_sensor_config()
        self.assertIsInstance(gl_config.renderer_class, RenderEngineGlParams)

        with self.assertRaisesRegex(ValueError, "show_rgbd_sensor"):
            mut.ModelVisualizer(show_rgbd_sensor="bad")

    def test_show_rgbd_sensor_availability_messaging(self):
        """Checks that RenderEngineGl is offered only when available."""
        self.addCleanup(importlib.reload, mut_private)

        with patch.object(mut_geometry, "kHasRenderEngineGl", True):
            model_visualizer_cls = importlib.reload(mut_private).ModelVisualizer
            self.assertEqual(
                model_visualizer_cls._supported_rgbd_renderers,
                ("vtk", "gl"),
            )
            dut = model_visualizer_cls(show_rgbd_sensor="gl")
            self.assertIsInstance(
                dut._make_rgbd_renderer_class(show_window=False),
                RenderEngineGlParams,
            )

        with patch.object(mut_geometry, "kHasRenderEngineGl", False):
            model_visualizer_cls = importlib.reload(mut_private).ModelVisualizer
            self.assertEqual(
                model_visualizer_cls._supported_rgbd_renderers, ("vtk",)
            )
            with self.assertRaisesRegex(ValueError, "kHasRenderEngineGl"):
                model_visualizer_cls(show_rgbd_sensor="gl")

    def test_show_rgbd_sensor_cli_help(self):
        """Checks that CLI help lists only supported renderers."""

        self.addCleanup(importlib.reload, mut_private)

        def get_help(*, has_render_engine_gl):
            stdout = io.StringIO()
            with (
                patch.object(
                    mut_geometry,
                    "kHasRenderEngineGl",
                    has_render_engine_gl,
                ),
                patch.object(
                    model_visualizer_cli,
                    "_ModelVisualizer",
                    importlib.reload(mut_private).ModelVisualizer,
                ),
                patch.object(sys, "argv", ["model_visualizer", "--help"]),
                redirect_stdout(stdout),
                self.assertRaisesRegex(SystemExit, "0"),
            ):
                model_visualizer_cli._main()
            return stdout.getvalue()

        with_gl_help = get_help(has_render_engine_gl=True)
        self.assertIn(
            "--show_rgbd_sensor [{BOOL,vtk,gl}]", with_gl_help
        )
        self.assertIn("Supported renderers: vtk, gl.", with_gl_help)
        no_gl_help = get_help(has_render_engine_gl=False)
        self.assertIn("--show_rgbd_sensor [{BOOL,vtk}]", no_gl_help)
        self.assertIn("Supported renderers: vtk.", no_gl_help)
        self.assertNotIn("--rgbd_renderer", no_gl_help)
        self.assertIn("kHasRenderEngineGl", no_gl_help)

    def test_show_rgbd_sensor_cli_values(self):
        """Checks the optional renderer value for --show_rgbd_sensor."""
        model_visualizer_cls = Mock()
        model_visualizer_cls._get_constructor_defaults.return_value = (
            mut.ModelVisualizer._get_constructor_defaults()
        )
        model_visualizer_cls._supported_rgbd_renderers = ("vtk", "gl")

        for argv, expected in [
            ([], None),
            (["--show_rgbd_sensor"], "vtk"),
            *(
                (["--show_rgbd_sensor", value], "vtk")
                for value in ("y", "yes", "t", "true", "on", "1", "TRUE")
            ),
            *(
                (["--show_rgbd_sensor", value], None)
                for value in ("n", "no", "f", "false", "off", "0", "FALSE")
            ),
            (["--show_rgbd_sensor", "vtk"], "vtk"),
            (["--show_rgbd_sensor", "gl"], "gl"),
            (["--show_rgbd_sensor=gl"], "gl"),
        ]:
            with (
                self.subTest(argv=argv),
                patch.object(
                    model_visualizer_cli,
                    "_ModelVisualizer",
                    model_visualizer_cls,
                ),
                patch.object(
                    sys,
                    "argv",
                    ["model_visualizer", *argv, "model.sdf"],
                ),
            ):
                model_visualizer_cli._main()
            self.assertEqual(
                model_visualizer_cls.call_args.kwargs["show_rgbd_sensor"],
                expected,
            )

    def test_show_rgbd_sensor_cli_unsupported_renderer(self):
        """Checks that an unavailable renderer is an option-value error."""
        model_visualizer_cls = Mock()
        model_visualizer_cls._get_constructor_defaults.return_value = (
            mut.ModelVisualizer._get_constructor_defaults()
        )
        model_visualizer_cls._supported_rgbd_renderers = ("vtk",)
        stderr = io.StringIO()
        with (
            patch.object(
                model_visualizer_cli,
                "_ModelVisualizer",
                model_visualizer_cls,
            ),
            patch.object(
                sys,
                "argv",
                [
                    "model_visualizer",
                    "--show_rgbd_sensor",
                    "gl",
                    "model.sdf",
                ],
            ),
            redirect_stderr(stderr),
            self.assertRaisesRegex(SystemExit, "2"),
        ):
            model_visualizer_cli._main()
        self.assertIn(
            "argument --show_rgbd_sensor: invalid value 'gl'; "
            "must be a boolean or one of: vtk",
            stderr.getvalue(),
        )

    def test_show_rgbd_sensor_cli_unknown_renderer(self):
        """Checks that an unknown renderer is an option-value error."""
        model_visualizer_cls = Mock()
        model_visualizer_cls._get_constructor_defaults.return_value = (
            mut.ModelVisualizer._get_constructor_defaults()
        )
        model_visualizer_cls._supported_rgbd_renderers = ("vtk", "gl")
        stderr = io.StringIO()
        with (
            patch.object(
                model_visualizer_cli,
                "_ModelVisualizer",
                model_visualizer_cls,
            ),
            patch.object(
                sys,
                "argv",
                [
                    "model_visualizer",
                    "--show_rgbd_sensor",
                    "bob",
                    "model.sdf",
                ],
            ),
            redirect_stderr(stderr),
            self.assertRaisesRegex(SystemExit, "2"),
        ):
            model_visualizer_cli._main()
        self.assertIn(
            "argument --show_rgbd_sensor: invalid value 'bob'; "
            "must be a boolean or one of: vtk, gl",
            stderr.getvalue(),
        )
