import os

import math
import sys
import textwrap
import unittest

from pydrake.common.test_utilities import numpy_compare
from pydrake.geometry.render import RenderLabel
from pydrake.math import RollPitchYaw, RigidTransform
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.sensors import (
    ImageDepth32F,
    ImageLabel16I,
    ImageRgba8U,
)
from pydrake.visualization import (
    ColorizeDepthImage,
    ColorizeLabelImage,
    ConcatenateImages,
    VideoWriter,
)


_PLATFORM_SUPPORTS_CV2 = "darwin" not in sys.platform


class TestColorizeDepthImage(unittest.TestCase):

    def test_smoke(self):
        """Runs all of the code once and spot checks a sample image."""
        dut = ColorizeDepthImage()
        context = dut.CreateDefaultContext()

        # Input a depth image with two finite values.
        depth = ImageDepth32F(6, 2, math.inf)
        depth.at(5, 0)[:] = 1.0
        depth.at(5, 1)[:] = 2.0
        dut.GetInputPort("depth_image_32f").FixValue(context, depth)

        # Expect a color image with matching values.
        # The closest pixel is white; the farthest pixel is black.
        default_invalid_color = [100, 0, 0, 255]
        expected_color = ImageRgba8U(6, 2, 0)
        expected_color.mutable_data[:] = default_invalid_color
        expected_color.at(5, 0)[:] = [255, 255, 255, 255]
        expected_color.at(5, 1)[:] = [0, 0, 0, 255]

        # Check the colorized image.
        actual_color = dut.GetOutputPort("color_image").Eval(context)
        numpy_compare.assert_allclose(
            actual_color.data, expected_color.data, atol=1)


class TestColorizeLabelImage(unittest.TestCase):

    def test_smoke(self):
        """Runs all of the code once and spot checks a sample image."""
        dut = ColorizeLabelImage()
        context = dut.CreateDefaultContext()

        # Input a label image with just one non-empty value.
        # Note that kEmpty is NOT a zero value; zero is the first real label.
        label = ImageLabel16I(6, 2, int(RenderLabel.kEmpty))
        label.at(5, 1)[:] = 0
        dut.GetInputPort("label_image").FixValue(context, label)

        # Expect a color image with a matching value.
        # Label #0 happens to be matplotlib "tableau blue" (#1F77B4).
        expected_color = ImageRgba8U(6, 2, 0)
        expected_color.at(5, 1)[:] = [31, 119, 180, 255]

        # Check the colorized image.
        actual_color = dut.GetOutputPort("color_image").Eval(context)
        numpy_compare.assert_allclose(
            actual_color.data, expected_color.data, atol=0)


class TestConcatenateImages(unittest.TestCase):

    def test_smoke(self):
        """Runs all of the code once and spot checks a sample image."""
        dut = ConcatenateImages(rows=2, cols=3)
        context = dut.CreateDefaultContext()

        # Set up the inputs.
        red_counter = 1
        for row in range(2):
            for col in range(3):
                image = ImageRgba8U(4, 2)
                image.at(0, 0)[:] = (red_counter, 0, 0, 255)
                dut.get_input_port(row=row, col=col).FixValue(context, image)
                red_counter += 1

        # Expect the red pixels in the correct order.
        expected = ImageRgba8U(12, 4, 0)
        expected.at(0, 0)[:] = (1, 0, 0, 255)
        expected.at(4, 0)[:] = (2, 0, 0, 255)
        expected.at(8, 0)[:] = (3, 0, 0, 255)
        expected.at(0, 2)[:] = (4, 0, 0, 255)
        expected.at(4, 2)[:] = (5, 0, 0, 255)
        expected.at(8, 2)[:] = (6, 0, 0, 255)

        # Check the actual image.
        actual = dut.GetOutputPort("color_image").Eval(context)
        numpy_compare.assert_allclose(
            actual.data, expected.data, atol=1)


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

        # Simulate for one second.
        diagram = builder.Build()
        simulator = Simulator(diagram)
        diagram_context = simulator.get_mutable_context()
        plant_context = plant.GetMyMutableContextFromRoot(diagram_context)
        plant.get_actuation_input_port().FixValue(
            plant_context, [0] * plant.num_positions())
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
