# -*- coding: utf-8 -*-

from pydrake.systems.rendering import (
    FrameVelocity,
    MultibodyPositionToGeometryPose,
    PoseBundle,
    PoseAggregator,
    PoseVector,
)

import copy
import unittest
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.common.value import AbstractValue
from pydrake.geometry import SceneGraph
from pydrake.math import RigidTransform
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.math import (
    SpatialVelocity,
)
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import (
    BasicVector,
    PortDataType,
)
from pydrake.common.eigen_geometry import Quaternion


def normalized(x):
    return x / np.linalg.norm(x)


class TestRendering(unittest.TestCase):
    # 2021-12-01 deprecation. Remove the whole test.
    def test_pose_vector(self):
        with catch_drake_warnings(expected_count=1):
            value = PoseVector()
        self.assertTrue(isinstance(value, BasicVector))
        self.assertTrue(isinstance(copy.copy(value), PoseVector))
        self.assertTrue(isinstance(value.Clone(), PoseVector))
        self.assertEqual(value.size(), PoseVector.kSize)
        # - Accessors.
        self.assertTrue(isinstance(value.get_transform(), RigidTransform))
        self.assertTrue(isinstance(
            value.get_rotation(), Quaternion))
        self.assertTrue(isinstance(
            value.get_translation(), np.ndarray))
        # - Value.
        self.assertTrue(np.allclose(
            value.get_transform().GetAsMatrix4(), np.eye(4, 4)))
        # - Mutators.
        p = [0, 1, 2]
        q = Quaternion(wxyz=normalized([0.1, 0.3, 0.7, 0.9]))
        X_expected = RigidTransform(quaternion=q, p=p)
        value.set_translation(p)
        value.set_rotation(q)
        self.assertTrue(np.allclose(
            value.get_transform().GetAsMatrix4(), X_expected.GetAsMatrix4()))
        # - Ensure ordering is ((px, py, pz), (qw, qx, qy, qz))
        vector_expected = np.hstack((p, q.wxyz()))
        vector_actual = value.get_value()
        self.assertTrue(np.allclose(vector_actual, vector_expected))
        # - Fully-parameterized constructor.

        with catch_drake_warnings(expected_count=1):
            value1 = PoseVector(rotation=q, translation=p)
        self.assertTrue(np.allclose(
            value1.get_transform().GetAsMatrix4(), X_expected.GetAsMatrix4()))
        # Test mutation via RigidTransform
        p2 = [10, 20, 30]
        q2 = Quaternion(wxyz=normalized([0.2, 0.3, 0.5, 0.8]))
        X2_expected = RigidTransform(quaternion=q2, p=p2)
        value.set_transform(X2_expected)
        self.assertTrue(np.allclose(
            value.get_transform().GetAsMatrix4(), X2_expected.GetAsMatrix4()))

    # 2021-12-01 deprecation. Remove the whole test.
    def test_frame_velocity(self):
        with catch_drake_warnings(expected_count=1):
            frame_velocity = FrameVelocity()
        self.assertTrue(isinstance(frame_velocity, BasicVector))
        self.assertTrue(isinstance(copy.copy(frame_velocity), FrameVelocity))
        self.assertTrue(isinstance(frame_velocity.Clone(), FrameVelocity))
        self.assertEqual(frame_velocity.size(), FrameVelocity.kSize)
        # - Accessors.
        self.assertTrue(isinstance(
            frame_velocity.get_velocity(), SpatialVelocity))
        # - Value.
        self.assertTrue(np.allclose(
            frame_velocity.get_velocity().rotational(), 3 * [0.]))
        self.assertTrue(np.allclose(
            frame_velocity.get_velocity().translational(), 3 * [0.]))
        # - Mutators.
        w = [0.1, 0.3, 0.5]
        v = [0., 1., 2.]
        frame_velocity.set_velocity(SpatialVelocity(w=w, v=v))
        self.assertTrue(np.allclose(
            frame_velocity.get_velocity().rotational(), w))
        self.assertTrue(np.allclose(
            frame_velocity.get_velocity().translational(), v))
        # - Ensure ordering is ((wx, wy, wz), (vx, vy, vz))
        velocity_expected = np.hstack((w, v))
        velocity_actual = frame_velocity.get_value()
        self.assertTrue(np.allclose(velocity_actual, velocity_expected))
        # - Fully-parameterized constructor.
        with catch_drake_warnings(expected_count=1):
            frame_velocity1 = FrameVelocity(SpatialVelocity(w=w, v=v))
        self.assertTrue(np.allclose(
            frame_velocity1.get_velocity().rotational(), w))
        self.assertTrue(np.allclose(
            frame_velocity1.get_velocity().translational(), v))

    # 2021-12-01 deprecation. Remove the whole test.
    def test_pose_bundle(self):
        num_poses = 7
        with catch_drake_warnings(expected_count=1):
            bundle = PoseBundle(num_poses)
        # - Accessors.
        self.assertEqual(bundle.get_num_poses(), num_poses)
        self.assertTrue(isinstance(bundle.get_transform(0), RigidTransform))
        self.assertTrue(isinstance(bundle.get_velocity(0), FrameVelocity))
        # - Mutators.
        kIndex = 5
        p = [0, 1, 2]
        q = Quaternion(wxyz=normalized([0.1, 0.3, 0.7, 0.9]))
        pose = RigidTransform(quaternion=q, p=p)
        bundle.set_transform(kIndex, pose)
        self.assertTrue((bundle.get_transform(kIndex).GetAsMatrix34()
                         == pose.GetAsMatrix34()).all())
        w = [0.1, 0.3, 0.5]
        v = [0., 1., 2.]

        with catch_drake_warnings(expected_count=1):
            frame_velocity = FrameVelocity(SpatialVelocity(w=w, v=v))
        bundle.set_velocity(kIndex, frame_velocity)
        vel_actual = bundle.get_velocity(kIndex).get_velocity()
        self.assertTrue(np.allclose(vel_actual.rotational(), w))
        self.assertTrue(np.allclose(vel_actual.translational(), v))
        name = "Alice"
        bundle.set_name(kIndex, name)
        self.assertEqual(bundle.get_name(kIndex), name)
        instance_id = 42  # Supply a random instance id.
        bundle.set_model_instance_id(kIndex, instance_id)
        self.assertEqual(bundle.get_model_instance_id(kIndex), instance_id)

    # 2021-12-01 deprecation. Remove the whole test.
    def test_pose_aggregator(self):
        with catch_drake_warnings(expected_count=1):
            aggregator = PoseAggregator()
        # - Set-up.
        instance_id1 = 5  # Supply a random instance id.
        port1 = aggregator.AddSingleInput("pose_only", instance_id1)
        self.assertEqual(port1.get_data_type(), PortDataType.kVectorValued)
        self.assertEqual(port1.size(), PoseVector.kSize)
        instance_id2 = 42  # Supply another random, but unique, id.
        ports2 = aggregator.AddSinglePoseAndVelocityInput(
            "pose_and_velocity", instance_id2)

        with catch_drake_warnings(expected_count=4):
            self.assertEqual(ports2.pose_input_port.get_data_type(),
                             PortDataType.kVectorValued)
            self.assertEqual(ports2.pose_input_port.size(), PoseVector.kSize)
            self.assertEqual(ports2.velocity_input_port.get_data_type(),
                             PortDataType.kVectorValued)
            self.assertEqual(ports2.velocity_input_port.size(),
                             FrameVelocity.kSize)
        num_poses = 1
        port3 = aggregator.AddBundleInput("pose_bundle", num_poses)
        self.assertEqual(port3.get_data_type(), PortDataType.kAbstractValued)

        # - CalcOutput.
        self.assertEqual(aggregator.get_output_port(0).get_data_type(),
                         PortDataType.kAbstractValued)
        context = aggregator.CreateDefaultContext()
        output = aggregator.AllocateOutput()

        p1 = [0, 1, 2]

        with catch_drake_warnings(expected_count=1):
            pose1 = PoseVector()
        pose1.set_translation(p1)
        p2 = [5, 7, 9]

        with catch_drake_warnings(expected_count=1):
            pose2 = PoseVector()
        pose2.set_translation(p2)
        w = [0.3, 0.4, 0.5]
        v = [0.5, 0.6, 0.7]

        with catch_drake_warnings(expected_count=1):
            velocity = FrameVelocity()
        velocity.set_velocity(SpatialVelocity(w=w, v=v))
        p3 = [50, 70, 90]
        q3 = Quaternion(wxyz=normalized([0.1, 0.3, 0.7, 0.9]))

        with catch_drake_warnings(expected_count=1):
            bundle = PoseBundle(num_poses)
        bundle.set_transform(0, RigidTransform(quaternion=q3, p=p3))
        bundle_value = AbstractValue.Make(bundle)

        aggregator.get_input_port(0).FixValue(context, pose1)
        aggregator.get_input_port(1).FixValue(context, pose2)
        aggregator.get_input_port(2).FixValue(context, velocity)
        aggregator.get_input_port(3).FixValue(context, bundle_value)

        aggregator.CalcOutput(context, output)

        value = output.get_data(0).get_value()
        self.assertEqual(value.get_num_poses(), 3)
        pose1_actual = RigidTransform(p=p1)
        self.assertTrue((value.get_transform(0).GetAsMatrix34()
                         == pose1_actual.GetAsMatrix34()).all())
        pose2_actual = RigidTransform(p=p2)
        self.assertTrue((value.get_transform(1).GetAsMatrix34()
                         == pose2_actual.GetAsMatrix34()).all())
        vel_actual = value.get_velocity(1).get_velocity()
        self.assertTrue(np.allclose(vel_actual.rotational(), w))
        self.assertTrue(np.allclose(vel_actual.translational(), v))
        pose3_actual = RigidTransform(quaternion=q3, p=p3)
        self.assertTrue((value.get_transform(2).GetAsMatrix34()
                         == pose3_actual.GetAsMatrix34()).all())

    def testMultibodyPositionToGeometryPose(self):
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        plant = MultibodyPlant(time_step=0.01)
        model_instance = Parser(plant).AddModelFromFile(file_name)
        scene_graph = SceneGraph()
        plant.RegisterAsSourceForSceneGraph(scene_graph)
        plant.Finalize()

        to_pose = MultibodyPositionToGeometryPose(
            plant=plant, input_multibody_state=False)

        # Check the size of the input.
        self.assertEqual(to_pose.get_input_port().size(), 2)

        # Just check the spelling of the output port (size is not meaningful
        # for Abstract-valued ports).
        to_pose.get_output_port()
