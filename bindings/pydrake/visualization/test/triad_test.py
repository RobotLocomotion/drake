import unittest

import numpy as np

from pydrake.geometry import Meshcat, Rgba
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.plant import MultibodyPlant
from pydrake.planning import RobotDiagramBuilder
import pydrake.visualization as mut


class TestTriad(unittest.TestCase):

    def setUp(self):
        # Create a simple robot.
        builder = RobotDiagramBuilder()
        builder.parser().AddModels(
            url="package://drake/multibody/benchmarks/acrobot/acrobot.sdf")
        robot = builder.Build()
        self._plant = robot.plant()
        self._scene_graph = robot.scene_graph()

    def test_via_body(self):
        # Illustrate the Link2 body frame.
        x, y, z = mut.AddFrameTriadIllustration(
            scene_graph=self._scene_graph,
            body=self._plant.GetBodyByName("Link2"),
            name="foo",
            length=0.2,
            radius=0.001,
            opacity=0.5,
            X_FT=RigidTransform())

        # Check the geometry pose of each axis.
        inspect = self._scene_graph.model_inspector()
        np.testing.assert_equal(
            inspect.GetPoseInFrame(x).GetAsMatrix34(),
            RigidTransform(
                RotationMatrix.MakeYRotation(np.pi/2),
                [0.1, 0, 0]).GetAsMatrix34())
        np.testing.assert_equal(
            inspect.GetPoseInFrame(y).GetAsMatrix34(),
            RigidTransform(
                RotationMatrix.MakeXRotation(np.pi/2),
                [0, 0.1, 0]).GetAsMatrix34())
        np.testing.assert_equal(
            inspect.GetPoseInFrame(z).GetAsMatrix34(),
            RigidTransform(
                RotationMatrix.MakeZRotation(np.pi/2),
                [0, 0, 0.1]).GetAsMatrix34())

        # Check the geometry details of each axis.
        for i, char in enumerate(("x", "y", "z")):
            geom_id = [x, y, z][i]
            frame_name = inspect.GetName(inspect.GetFrameId(geom_id))
            self.assertEqual(frame_name, "acrobot::Link2")
            self.assertEqual(inspect.GetName(geom_id), f"foo {char}-axis")
            self.assertEqual(inspect.GetShape(geom_id).length(), 0.2)
            self.assertEqual(inspect.GetShape(geom_id).radius(), 0.001)
            props = inspect.GetIllustrationProperties(geom_id)
            self.assertEqual(props.GetProperty("phong", "diffuse"),
                             Rgba(r=(i == 0), g=(i == 1), b=(i == 2), a=0.5))

    def test_via_frame_id(self):
        # Illustrate the Link2 geometry frame.
        body = self._plant.GetBodyByName("Link2")
        expected_frame = self._plant.GetBodyFrameIdIfExists(body.index())
        geom_ids = mut.AddFrameTriadIllustration(
            plant=self._plant,
            scene_graph=self._scene_graph,
            frame_id=expected_frame)

        # Check that all three geoms were added.
        self.assertEqual(len(geom_ids), 3)
        inspect = self._scene_graph.model_inspector()
        for geom_id in geom_ids:
            actual_frame = inspect.GetFrameId(geom_id)
            self.assertEqual(actual_frame, expected_frame)

    def test_redundant_plant_arg(self):
        # Pass a plant= even though it's not needed.
        # It's sliently cross-checked and accepted.
        mut.AddFrameTriadIllustration(
            plant=self._plant,
            scene_graph=self._scene_graph,
            body=self._plant.GetBodyByName("Link2"))

    def test_wrong_plant_arg(self):
        # Pass the wrong plant= value and get rejected.
        with self.assertRaisesRegex(ValueError, "Mismatched"):
            mut.AddFrameTriadIllustration(
                plant=MultibodyPlant(0.0),
                scene_graph=self._scene_graph,
                body=self._plant.GetBodyByName("Link2"))
