import pydrake.visualization as mut

import logging
from pathlib import Path
import tempfile
import unittest

import numpy as np
import umsgpack

import pydrake.common.test_utilities.numpy_compare as numpy_compare
from pydrake.geometry import Meshcat
from pydrake.math import (
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
)


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
        meshcat._InjectWebsocketMessage(message=umsgpack.packb({
            "type": "camera_pose",
            "camera_pose": [
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                1, 2, 3, 1,
            ],
            "is_perspective": True,
        }))
        # Transform y-up to z-up, and from facing in the +z direction to the -z
        # direction (with concomitant flip of the y-axis).
        X_WB_expected = RigidTransform(
            R=RotationMatrix(RollPitchYaw(np.pi / 2, np.pi, np.pi)),
            p=[1.0, -3.0, 2.0])

        # N.B. We don't need perception geometry in the scene -- we'll rely on
        # the RgbdSensor unit tests to check that cameras work as advertised.
        # Our only goal here is to achieve statement-level code coverage of the
        # ModelVisualizer code when show_rgbd_sensor=True.
        model = """<?xml version="1.0"?>
          <sdf version="1.9">
            <model name="sample">
              <link name="base"/>
            </model>
          </sdf>
        """
        dut = mut.ModelVisualizer(meshcat=meshcat, show_rgbd_sensor=True)
        dut.parser().AddModelsFromString(model, "sdf")
        dut.Run(loop_once=True)

        # Confirm that the pose got updated properly. Updated pose is a proxy
        # for the full behavior in dut._render_if_necessary().
        camera_frame = dut._diagram.plant().GetFrameByName(
            "$rgbd_sensor_offset")
        X_WB = camera_frame.GetPoseInParentFrame(
            dut._diagram.plant().GetMyContextFromRoot(
                dut._context))

        numpy_compare.assert_allclose(X_WB.GetAsMatrix34(),
                                      X_WB_expected.GetAsMatrix34(),
                                      atol=1e-15)

    def test_camera_with_config(self):
        # There are various additional pieces of logic that *could* be tested.
        # For example, making sure that the X_PB values in the config file
        # get ignored (in lieu of configuring the render camera to track the
        # viewer camera). We're not testing these because if there are errors,
        # they would become apparent during use. (E.g., the renderer would stop
        # tracking the viewer's camera). These tests serve as a regression test
        # against *not* consuming a camera configuration or messages in the
        # event that parsing fails.

        # Create a config file which configures the camera differently from
        # the default.
        renderer_name = "__unique_test_renderer__"
        scratch_dir = Path(tempfile.mkdtemp())
        config_file = scratch_dir / "test_config.yaml"
        with open(config_file, "w") as f:
            f.write(f"""
renderer_name: {renderer_name}
renderer_class: RenderEngineVtk
width: 512
height: 256
""")
        # If we don't provide a configuration file, we don't get a renderer
        # with the expected name.
        dut = mut.ModelVisualizer(show_rgbd_sensor=True)
        dut.Finalize()
        scene_graph = dut._diagram.scene_graph()
        self.assertTrue(scene_graph.HasRenderer("default"))
        self.assertFalse(scene_graph.HasRenderer(renderer_name))

        # Providing a configuration file uses that file.
        dut = mut.ModelVisualizer(show_rgbd_sensor=True,
                                  camera_config=config_file)
        dut.Finalize()
        scene_graph = dut._diagram.scene_graph()
        self.assertFalse(scene_graph.HasRenderer("default"))
        self.assertTrue(scene_graph.HasRenderer(renderer_name))

        # Config without requesting an rgbd sensor has no effect.
        dut = mut.ModelVisualizer(show_rgbd_sensor=False,
                                  camera_config=config_file)
        dut.Finalize()
        scene_graph = dut._diagram.scene_graph()
        self.assertFalse(scene_graph.HasRenderer("default"))
        self.assertFalse(scene_graph.HasRenderer(renderer_name))

        # Various kinds of config loading errors produce a default camera.
        # We'll use IOError as representative by passing an invalid path.
        dut = mut.ModelVisualizer(show_rgbd_sensor=True,
                                  camera_config=config_file / ".bad")
        with self.assertLogs("drake", logging.WARNING) as cm:
            dut.Finalize()
        self.assertEqual(len(cm.records), 1)
        self.assertRegex(cm.records[0].getMessage(),
                         "Error reading camera config file.*")

        scene_graph = dut._diagram.scene_graph()
        self.assertTrue(scene_graph.HasRenderer("default"))
        self.assertFalse(scene_graph.HasRenderer(renderer_name))
