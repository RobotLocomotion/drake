import os

import cv2
import numpy as np
import unittest

from pydrake.common import FindResourceOrThrow
from pydrake.math import RollPitchYaw, RigidTransform
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization.video import VideoWriter


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
