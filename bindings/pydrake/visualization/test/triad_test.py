import pydrake.visualization as mut  # ruff: isort: skip

import unittest

import numpy as np

from pydrake.geometry import Rgba
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import FixedOffsetFrame
from pydrake.planning import RobotDiagramBuilder


class TestTriad(unittest.TestCase):
    def setUp(self):
        # Create a simple robot.
        builder = RobotDiagramBuilder()
        builder.parser().AddModels(
            url="package://drake/multibody/benchmarks/acrobot/acrobot.sdf"
        )
        self._X_PF = RigidTransform([1.0, 2.0, 3.0])
        self._frame = builder.plant().AddFrame(
            FixedOffsetFrame(
                "frame", builder.plant().GetFrameByName("Link2"), self._X_PF
            )
        )
        self._frame_index = self._frame.index()
        robot = builder.Build()
        self._plant = robot.plant()
        self._scene_graph = robot.scene_graph()

        self._axis_offsets = {
            "x": RigidTransform(
                RotationMatrix.MakeYRotation(np.pi / 2), [0.1, 0, 0]
            ),
            "y": RigidTransform(
                RotationMatrix.MakeXRotation(np.pi / 2), [0, 0.1, 0]
            ),
            "z": RigidTransform(
                RotationMatrix.MakeZRotation(np.pi / 2), [0, 0, 0.1]
            ),
        }

    def _assert_equal_transform(self, X1, X2):
        np.testing.assert_equal(X1.GetAsMatrix34(), X2.GetAsMatrix34())

    def test_via_body(self):
        # Illustrate the Link2 body frame.
        x, y, z = mut.AddFrameTriadIllustration(
            scene_graph=self._scene_graph,
            body=self._plant.GetBodyByName("Link2"),
            name="foo",
            length=0.2,
            radius=0.001,
            opacity=0.5,
            X_FT=RigidTransform(),
        )

        # Check the geometry pose of each axis.
        inspect = self._scene_graph.model_inspector()
        self._assert_equal_transform(
            inspect.GetPoseInFrame(x), self._axis_offsets["x"]
        )
        self._assert_equal_transform(
            inspect.GetPoseInFrame(y), self._axis_offsets["y"]
        )
        self._assert_equal_transform(
            inspect.GetPoseInFrame(z), self._axis_offsets["z"]
        )

        # Check the geometry details of each axis.
        for i, char in enumerate(("x", "y", "z")):
            geom_id = [x, y, z][i]
            frame_name = inspect.GetName(inspect.GetFrameId(geom_id))
            self.assertEqual(frame_name, "acrobot::Link2")
            self.assertEqual(
                inspect.GetName(geom_id), f"_frames::foo::{char}-axis"
            )
            self.assertEqual(inspect.GetShape(geom_id).length(), 0.2)
            self.assertEqual(inspect.GetShape(geom_id).radius(), 0.001)
            props = inspect.GetIllustrationProperties(geom_id)
            self.assertEqual(
                props.GetProperty("phong", "diffuse"),
                Rgba(r=(i == 0), g=(i == 1), b=(i == 2), a=0.5),
            )

    def test_via_frame(self):
        # Illustrate our custom added frame.
        X_FT = RigidTransform([0.1, 0.2, 0.3])
        x, y, z = mut.AddFrameTriadIllustration(
            scene_graph=self._scene_graph,
            frame=self._frame,
            length=0.2,
            X_FT=X_FT,
        )

        # Check the geometry pose of each axis.
        inspect = self._scene_graph.model_inspector()
        X_PT = self._X_PF @ X_FT
        self._assert_equal_transform(
            inspect.GetPoseInFrame(x), X_PT @ self._axis_offsets["x"]
        )
        self._assert_equal_transform(
            inspect.GetPoseInFrame(y), X_PT @ self._axis_offsets["y"]
        )
        self._assert_equal_transform(
            inspect.GetPoseInFrame(z), X_PT @ self._axis_offsets["z"]
        )

    def test_via_frame_index(self):
        # Illustrate our custom added frame.
        X_FT = RigidTransform([0.1, 0.2, 0.3])
        x, y, z = mut.AddFrameTriadIllustration(
            plant=self._plant,
            scene_graph=self._scene_graph,
            frame_index=self._frame_index,
            length=0.2,
            X_FT=X_FT,
        )

        # Check the geometry pose of each axis.
        inspect = self._scene_graph.model_inspector()
        X_PT = self._X_PF @ X_FT
        self._assert_equal_transform(
            inspect.GetPoseInFrame(x), X_PT @ self._axis_offsets["x"]
        )
        self._assert_equal_transform(
            inspect.GetPoseInFrame(y), X_PT @ self._axis_offsets["y"]
        )
        self._assert_equal_transform(
            inspect.GetPoseInFrame(z), X_PT @ self._axis_offsets["z"]
        )

    def test_via_frame_id(self):
        # Illustrate the Link2 geometry frame.
        body = self._plant.GetBodyByName("Link2")
        expected_frame = self._plant.GetBodyFrameIdIfExists(body.index())
        geom_ids = mut.AddFrameTriadIllustration(
            plant=self._plant,
            scene_graph=self._scene_graph,
            frame_id=expected_frame,
        )

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
            body=self._plant.GetBodyByName("Link2"),
        )
        mut.AddFrameTriadIllustration(
            plant=self._plant,
            scene_graph=self._scene_graph,
            frame=self._frame,
            name="frame 2",
        )

    def test_wrong_plant_arg(self):
        # Pass the wrong plant= value and get rejected.
        wrong_plant = MultibodyPlant(0.0)
        with self.assertRaisesRegex(ValueError, "Mismatched body="):
            mut.AddFrameTriadIllustration(
                plant=wrong_plant,
                scene_graph=self._scene_graph,
                body=self._plant.GetBodyByName("Link2"),
            )
        with self.assertRaisesRegex(ValueError, "Mismatched frame="):
            mut.AddFrameTriadIllustration(
                plant=wrong_plant,
                scene_graph=self._scene_graph,
                frame=self._frame,
            )
