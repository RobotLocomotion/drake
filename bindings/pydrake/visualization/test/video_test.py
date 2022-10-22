import os

import cv2
import numpy as np
import unittest

from pydrake.common import FindResourceOrThrow
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


class TestColorizeDepthImage(unittest.TestCase):

    def test_smoke(self):
        dut = ColorizeDepthImage()
        context = dut.CreateDefaultContext()

        # Input a depth image with two finite values.
        depth = ImageDepth32F(6, 2, np.inf)
        depth.at(5, 0)[:] = 1.0
        depth.at(5, 1)[:] = 2.0
        dut.GetInputPort("depth_image_32f").FixValue(context, depth)

        # Expect a color image with matching values.
        # The closest pixel is white; the furthest pixcel is black.
        expected_color = ImageRgba8U(6, 2, 0)
        expected_color.mutable_data[:] = dut.invalid_color
        expected_color.at(5, 0)[:] = [255, 255, 255, 255]
        expected_color.at(5, 1)[:] = [0, 0, 0, 255]

        # Check the colorized image.
        actual_color = dut.GetOutputPort("color_image").Eval(context)
        numpy_compare.assert_allclose(
            actual_color.data, expected_color.data, atol=1)


class TestColorizeLabelImage(unittest.TestCase):

    def test_smoke(self):
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
        print(repr(actual.data))
        print(repr(expected.data))
        numpy_compare.assert_allclose(
            actual.data, expected.data, atol=1)


class TestVideoWriter(unittest.TestCase):

    def _test_usage(self, filename, kinds):
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(
            builder, time_step=0.0)
        parser = Parser(plant)
        iiwa_file = FindResourceOrThrow(
            "drake/manipulation/models/"
            "iiwa_description/sdf/iiwa14_no_collision.sdf")
        wsg_file = FindResourceOrThrow(
            "drake/manipulation/models/"
            "wsg_50_description/sdf/schunk_wsg_50.sdf")
        iiwa_model = parser.AddModelFromFile(iiwa_file)
        wsg_model = parser.AddModelFromFile(wsg_file)
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName("iiwa_link_0", iiwa_model))
        plant.WeldFrames(
            plant.GetFrameByName("iiwa_link_7", iiwa_model),
            plant.GetFrameByName("body", wsg_model))
        plant.Finalize()

        # Add the video writer.
        sensor_pose = RigidTransform(
            RollPitchYaw([-np.pi/2, 0, np.pi/2]), [2, 0, 0.75])
        writer = VideoWriter.AddToBuilder(
            filename=filename, builder=builder, sensor_pose=sensor_pose,
            kinds=kinds)

        # Simulate.
        diagram = builder.Build()
        simulator = Simulator(diagram)
        diagram_context = simulator.get_mutable_context()
        plant_context = plant.GetMyMutableContextFromRoot(diagram_context)
        plant.get_actuation_input_port().FixValue(
            plant_context, [0] * plant.num_positions())
        simulator.AdvanceTo(1.0)
        writer.Save()

        # Check the results.
        readback = cv2.VideoCapture(filename)
        self.assertEqual(readback.get(cv2.CAP_PROP_FPS), 16)
        self.assertEqual(readback.get(cv2.CAP_PROP_FRAME_COUNT), 17)

    def test_typical_usage(self):
        filename = os.environ["TEST_UNDECLARED_OUTPUTS_DIR"] + "/color.mp4"
        self._test_usage(filename, ("color",))

    def test_multikind(self):
        filename = os.environ["TEST_UNDECLARED_OUTPUTS_DIR"] + "/multi.mp4"
        self._test_usage(filename, ("color", "depth", "label"))
