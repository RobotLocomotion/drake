import math
import os
import sys
import textwrap
import unittest

from pydrake.math import RollPitchYaw, RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import VideoWriter


_PLATFORM_SUPPORTS_CV2 = "darwin" not in sys.platform


class TestVideoWriter(unittest.TestCase):
    """Tests for VideoWriter.

    To view the output videos from the test cases, you can do:
      unzip bazel-testlogs/bindings/pydrake/visualization/py/video_test/test.outputs/outputs.zip  # noqa
    """

    @staticmethod
    def _cv2():
        """On some platforms, we don't have OpenCV as a dependency so we can't
        import it atop this file as would be usual.
        """
        if _PLATFORM_SUPPORTS_CV2:
            import cv2
            return cv2
        else:
            return None

    def _test_usage(self, filename, backend, kinds):
        """Runs through the typical usage and checks that a well-formed video
        output file was created on disk.
        """
        builder = DiagramBuilder()
        plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
        models = textwrap.dedent("""
        directives:
        - add_model:
            name: iiwa
            file: package://drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf  # noqa
        - add_model:
            name: wsg
            file: package://drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf  # noqa
        - add_weld:
            parent: world
            child: iiwa::iiwa_link_0
        - add_weld:
            parent: iiwa::iiwa_link_7
            child: wsg::body
        """)
        Parser(plant).AddModelsFromString(models, "dmd.yaml")
        plant.Finalize()

        # Add the video writer.
        fps = 16 if backend == "cv2" else 10
        sensor_pose = RigidTransform(
            RollPitchYaw([-math.pi/2, 0, math.pi/2]), [2, 0, 0.75])
        writer = VideoWriter.AddToBuilder(
            filename=filename, builder=builder, sensor_pose=sensor_pose,
            fps=fps, kinds=kinds, backend=backend)

        # Simulate for one second (add torque to the plant to make it move).
        diagram = builder.Build()
        simulator = Simulator(diagram)
        diagram_context = simulator.get_mutable_context()
        plant_context = plant.GetMyMutableContextFromRoot(diagram_context)
        plant.get_actuation_input_port().FixValue(
            plant_context, [10] * plant.num_positions())
        simulator.AdvanceTo(1.0)
        writer.Save()

        # The video file should have been created, with non-trivial size.
        self.assertGreater(os.path.getsize(filename), 5000)

        # Check that the video can be loaded, and has the correct fps and
        # number of frames for a 1-second simulation.
        if _PLATFORM_SUPPORTS_CV2:
            cv2 = self._cv2()
            readback = cv2.VideoCapture(filename)
            self.assertEqual(readback.get(cv2.CAP_PROP_FRAME_COUNT), fps + 1)
            self.assertEqual(readback.get(cv2.CAP_PROP_FPS), fps)

    def test_pil_color_only(self):
        """Tests PIL (gif) output of a color-only camera."""
        filename = os.environ["TEST_UNDECLARED_OUTPUTS_DIR"] + "/color.gif"
        self._test_usage(filename, "PIL", ("color",))

    def test_pil_everything(self):
        """Tests PIL (gif) output of a color+depth+label."""
        filename = os.environ["TEST_UNDECLARED_OUTPUTS_DIR"] + "/multi.gif"
        self._test_usage(filename, "PIL", ("color", "depth", "label"))

    @unittest.skipUnless(_PLATFORM_SUPPORTS_CV2, "Not tested on this platform")
    def test_cv2_color_only(self):
        """Tests cv2 (mp4) output of a color-only camera."""
        filename = os.environ["TEST_UNDECLARED_OUTPUTS_DIR"] + "/color.mp4"
        self._test_usage(filename, "cv2", ("color",))

    @unittest.skipUnless(_PLATFORM_SUPPORTS_CV2, "Not tested on this platform")
    def test_cv2_everything(self):
        """Tests cv2 (mp4) output of a color+depth+label."""
        filename = os.environ["TEST_UNDECLARED_OUTPUTS_DIR"] + "/multi.mp4"
        self._test_usage(filename, "cv2", ("color", "depth", "label"))

    @unittest.skipUnless(_PLATFORM_SUPPORTS_CV2, "Not tested on this platform")
    def test_cv2_bad_fourcc(self):
        """Tests cv2 sanity checking of fourcc."""
        builder = DiagramBuilder()
        AddMultibodyPlantSceneGraph(builder, time_step=0.0)
        filename = os.environ["TEST_UNDECLARED_OUTPUTS_DIR"] + "/bad.mp4"
        with self.assertRaisesRegex(ValueError, "wrong.*must be"):
            VideoWriter.AddToBuilder(
                filename=filename, builder=builder,
                sensor_pose=RigidTransform(), fps=16, backend="cv2",
                fourcc="wrong")

    def test_bad_backend(self):
        """Tests detection of a malformed backend setting."""
        builder = DiagramBuilder()
        AddMultibodyPlantSceneGraph(builder, time_step=0.0)
        with self.assertRaises(Exception) as cm:
            VideoWriter.AddToBuilder(
                filename="file", builder=builder, sensor_pose=RigidTransform(),
                backend="WRONG")
        self.assertIn("WRONG", str(cm.exception))

    def test_bad_backend(self):
        """Tests detection of a malformed kinds setting."""
        builder = DiagramBuilder()
        AddMultibodyPlantSceneGraph(builder, time_step=0.0)
        with self.assertRaises(Exception) as cm:
            VideoWriter.AddToBuilder(
                filename="file", builder=builder, sensor_pose=RigidTransform(),
                kinds=("WRONG",))
        self.assertIn("WRONG", str(cm.exception))
