# -*- coding: utf-8 -*-

from pydrake.systems.rendering import (
    FrameVelocity,
    PoseBundle,
    PoseAggregator,
    PoseVector,
)

import copy
import unittest
import numpy as np

from pydrake.multibody.multibody_tree.math import (
    SpatialVelocity,
)
from pydrake.systems.framework import (
    AbstractValue,
    BasicVector,
    PortDataType,
)
from pydrake.util.eigen_geometry import (
    Isometry3,
    Quaternion,
)


def normalized(x):
    return x / np.linalg.norm(x)


class TestRendering(unittest.TestCase):
    def test_pose_vector(self):
        value = PoseVector()
        self.assertTrue(isinstance(value, BasicVector))
        self.assertTrue(isinstance(copy.copy(value), PoseVector))
        self.assertTrue(isinstance(value.Clone(), PoseVector))
        self.assertEqual(value.size(), PoseVector.kSize)
        # - Accessors.
        self.assertTrue(isinstance(
            value.get_isometry(), Isometry3))
        self.assertTrue(isinstance(
            value.get_rotation(), Quaternion))
        self.assertTrue(isinstance(
            value.get_translation(), np.ndarray))
        # - Value.
        self.assertTrue(np.allclose(
            value.get_isometry().matrix(), np.eye(4, 4)))
        # - Mutators.
        p = [0, 1, 2]
        q = Quaternion(wxyz=normalized([0.1, 0.3, 0.7, 0.9]))
        X_expected = Isometry3(q, p)
        value.set_translation(p)
        value.set_rotation(q)
        self.assertTrue(np.allclose(
            value.get_isometry().matrix(), X_expected.matrix()))
        # - Ensure ordering is ((px, py, pz), (qw, qx, qy, qz))
        vector_expected = np.hstack((p, q.wxyz()))
        vector_actual = value.get_value()
        self.assertTrue(np.allclose(vector_actual, vector_expected))
        # - Fully-parameterized constructor.
        value1 = PoseVector(rotation=q, translation=p)
        self.assertTrue(np.allclose(
            value1.get_isometry().matrix(), X_expected.matrix()))

    def test_frame_velocity(self):
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
        frame_velocity1 = FrameVelocity(SpatialVelocity(w=w, v=v))
        self.assertTrue(np.allclose(
            frame_velocity1.get_velocity().rotational(), w))
        self.assertTrue(np.allclose(
            frame_velocity1.get_velocity().translational(), v))

    def test_pose_bundle(self):
        num_poses = 7
        bundle = PoseBundle(num_poses)
        # - Accessors.
        self.assertEqual(bundle.get_num_poses(), num_poses)
        self.assertTrue(isinstance(bundle.get_pose(0), Isometry3))
        self.assertTrue(isinstance(bundle.get_velocity(0), FrameVelocity))
        # - Mutators.
        kIndex = 5
        p = [0, 1, 2]
        q = Quaternion(wxyz=normalized([0.1, 0.3, 0.7, 0.9]))
        bundle.set_pose(kIndex, Isometry3(q, p))
        w = [0.1, 0.3, 0.5]
        v = [0., 1., 2.]
        frame_velocity = FrameVelocity(SpatialVelocity(w=w, v=v))
        bundle.set_velocity(kIndex, frame_velocity)
        self.assertTrue((bundle.get_pose(kIndex).matrix() ==
                         Isometry3(q, p).matrix()).all())
        vel_actual = bundle.get_velocity(kIndex).get_velocity()
        self.assertTrue(np.allclose(vel_actual.rotational(), w))
        self.assertTrue(np.allclose(vel_actual.translational(), v))
        name = "Alice"
        bundle.set_name(kIndex, name)
        self.assertEqual(bundle.get_name(kIndex), name)
        instance_id = 42  # Supply a random instance id.
        bundle.set_model_instance_id(kIndex, instance_id)
        self.assertEqual(bundle.get_model_instance_id(kIndex), instance_id)

    def test_pose_aggregator(self):
        aggregator = PoseAggregator()
        # - Set-up.
        instance_id1 = 5  # Supply a random instance id.
        port1 = aggregator.AddSingleInput("pose_only", instance_id1)
        self.assertEqual(port1.get_data_type(), PortDataType.kVectorValued)
        self.assertEqual(port1.size(), PoseVector.kSize)
        instance_id2 = 42  # Supply another random, but unique, id.
        ports2 = aggregator.AddSinglePoseAndVelocityInput(
            "pose_and_velocity", instance_id2)
        self.assertEqual(ports2.pose_descriptor.get_data_type(),
                         PortDataType.kVectorValued)
        self.assertEqual(ports2.pose_descriptor.size(), PoseVector.kSize)
        self.assertEqual(ports2.velocity_descriptor.get_data_type(),
                         PortDataType.kVectorValued)
        self.assertEqual(ports2.velocity_descriptor.size(),
                         FrameVelocity.kSize)
        num_poses = 1
        port3 = aggregator.AddBundleInput("pose_bundle", num_poses)
        self.assertEqual(port3.get_data_type(), PortDataType.kAbstractValued)

        # - CalcOutput.
        context = aggregator.CreateDefaultContext()
        output = aggregator.AllocateOutput(context)

        p1 = [0, 1, 2]
        pose1 = PoseVector()
        pose1.set_translation(p1)
        p2 = [5, 7, 9]
        pose2 = PoseVector()
        pose2.set_translation(p2)
        w = [0.3, 0.4, 0.5]
        v = [0.5, 0.6, 0.7]
        velocity = FrameVelocity()
        velocity.set_velocity(SpatialVelocity(w=w, v=v))
        p3 = [50, 70, 90]
        q3 = Quaternion(wxyz=normalized([0.1, 0.3, 0.7, 0.9]))
        bundle = PoseBundle(num_poses)
        bundle.set_pose(0, Isometry3(q3, p3))
        bundle_value = AbstractValue.Make(bundle)

        context.FixInputPort(0, pose1)
        context.FixInputPort(1, pose2)
        context.FixInputPort(2, velocity)
        context.FixInputPort(3, bundle_value)

        aggregator.CalcOutput(context, output)

        value = output.get_data(0).get_value()
        self.assertEqual(value.get_num_poses(), 3)
        isom1_actual = Isometry3()
        isom1_actual.set_translation(p1)
        self.assertTrue(
            (value.get_pose(0).matrix() == isom1_actual.matrix()).all())
        isom2_actual = Isometry3()
        isom2_actual.set_translation(p2)
        self.assertTrue(
            (value.get_pose(1).matrix() == isom2_actual.matrix()).all())
        vel_actual = value.get_velocity(1).get_velocity()
        self.assertTrue(np.allclose(vel_actual.rotational(), w))
        self.assertTrue(np.allclose(vel_actual.translational(), v))
        self.assertTrue(
            (value.get_pose(2).matrix() == Isometry3(q3, p3).matrix()).all())
