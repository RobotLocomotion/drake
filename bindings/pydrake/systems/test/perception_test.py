# -*- coding: utf-8 -*-

import unittest
import numpy as np

from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.perception import BaseField, Fields, PointCloud
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.perception import (
    PointCloudConcatenation, _ConcatenatePointClouds, _TileColors,
    _TransformPoints)


class TestConcatenatePointClouds(unittest.TestCase):
    def setUp(self):
        self.points_0 = np.array([[1.0], [2.0], [3.0]])
        self.colors_0 = np.array([[0], [128], [255]])

        self.points_1 = np.array([[4.0], [5.0], [6.0]])
        self.colors_1 = np.array([[50], [100], [200]])

        self.points_dict = {"0": self.points_0, "1": self.points_1}
        self.colors_dict = {"0": self.colors_0, "1": self.colors_1}

    def test_concatenation(self):
        scene_points, scene_colors = _ConcatenatePointClouds(
            self.points_dict, self.colors_dict)

        self.assertEqual(scene_points.shape, (3, len(self.points_dict)))
        self.assertEqual(scene_colors.shape, (3, len(self.colors_dict)))
        self.assertEqual(scene_points.shape, scene_colors.shape)

        for i, value in enumerate(self.points_0.flatten()):
            self.assertTrue(value in scene_points[i, :])

        for i, value in enumerate(self.points_1.flatten()):
            self.assertTrue(value in scene_points[i, :])

        for i, value in enumerate(self.colors_0.flatten()):
            self.assertTrue(value in scene_colors[i, :])

        for i, value in enumerate(self.colors_0.flatten()):
            self.assertTrue(value in scene_colors[i, :])


class TestTileColors(unittest.TestCase):
    def setUp(self):
        self.red = [255, 0, 0]
        self.blue = [0, 0, 255]

    def test_one_dim(self):
        tiled = _TileColors(self.red, 1)
        expected_tiled = np.array([[255], [0], [0]])
        self.assertTrue(np.allclose(tiled, expected_tiled))

    def test_three_dims(self):
        tiled = _TileColors(self.blue, 1)
        expected_tiled = np.array([[0, 0, 0], [0, 0, 0], [255, 255, 255]])
        self.assertTrue(np.allclose(tiled, expected_tiled))


class TestTransformPoints(unittest.TestCase):
    def setUp(self):
        self.points = np.array([[1, 1, 0], [2, 1, 0]]).T
        self.translation = RigidTransform(p=[1, 2, 3])
        self.rotation = RigidTransform(
            RotationMatrix(RollPitchYaw(0, 0, np.pi/2)))

    def test_translation(self):
        transformed_points = _TransformPoints(
            self.points, self.translation.GetAsMatrix4())
        expected_translated_points = np.array([[2, 3, 3], [3, 3, 3]]).T

        self.assertTrue(
            np.allclose(transformed_points, expected_translated_points))

    def test_rotation(self):
        transformed_points = _TransformPoints(
            self.points, self.rotation.GetAsMatrix4())
        expected_rotated_points = np.array([[-1, 1, 0], [-1, 2, 0]]).T

        self.assertTrue(
            np.allclose(transformed_points, expected_rotated_points))


class TestPointCloudConcatenation(unittest.TestCase):
    def setUp(self):
        builder = DiagramBuilder()

        X_WP_0 = RigidTransform.Identity()
        X_WP_1 = RigidTransform.Identity()
        X_WP_1.set_translation([1.0, 0, 0])

        id_list = ["0", "1"]

        self.pc_concat = builder.AddSystem(PointCloudConcatenation(id_list))

        self.num_points = 10000
        xyzs = np.random.uniform(-0.1, 0.1, (3, self.num_points))
        # Only go to 254 to distinguish between point clouds with and without
        # color.
        rgbs = np.random.uniform(0., 254.0, (3, self.num_points))

        self.pc = PointCloud(
            self.num_points,
            Fields(BaseField.kXYZs | BaseField.kRGBs))
        self.pc.mutable_xyzs()[:] = xyzs
        self.pc.mutable_rgbs()[:] = rgbs

        self.pc_no_rgbs = PointCloud(
            self.num_points, Fields(BaseField.kXYZs))
        self.pc_no_rgbs.mutable_xyzs()[:] = xyzs

        diagram = builder.Build()

        simulator = Simulator(diagram)

        self.context = diagram.GetMutableSubsystemContext(
            self.pc_concat, simulator.get_mutable_context())

        self.pc_concat.GetInputPort("X_FCi_0").FixValue(self.context, X_WP_0)
        self.pc_concat.GetInputPort("X_FCi_1").FixValue(self.context, X_WP_1)

    def test_no_rgb(self):
        self.pc_concat.GetInputPort("point_cloud_CiSi_0").FixValue(
            self.context, AbstractValue.Make(self.pc_no_rgbs))
        self.pc_concat.GetInputPort("point_cloud_CiSi_1").FixValue(
            self.context, AbstractValue.Make(self.pc_no_rgbs))

        fused_pc = self.pc_concat.GetOutputPort("point_cloud_FS").Eval(
            self.context)

        self.assertEqual(fused_pc.size(), 2 * self.num_points)

        # The first point cloud should be from [-0.1 to 0.1].
        # The second point cloud should be from [0.9 to 1.1].
        self.assertTrue(np.max(fused_pc.xyzs()[0, :]) >= 1.0)
        self.assertTrue(np.min(fused_pc.xyzs()[0, :]) <= 0.0)

        # Even if both input point clouds don't have rgbs, the fused point
        # cloud should contain rgbs of the default color.
        self.assertTrue(fused_pc.has_rgbs())
        self.assertTrue(
            np.all(fused_pc.rgbs()[:, 0] == np.array([255, 255, 255])))
        self.assertTrue(
            np.all(fused_pc.rgbs()[:, -1] == np.array([255, 255, 255])))

    def test_rgb(self):
        self.pc_concat.GetInputPort("point_cloud_CiSi_0").FixValue(
            self.context, self.pc)
        self.pc_concat.GetInputPort("point_cloud_CiSi_1").FixValue(
            self.context, self.pc)

        fused_pc = self.pc_concat.GetOutputPort("point_cloud_FS").Eval(
            self.context)

        self.assertEqual(fused_pc.size(), 2 * self.num_points)

        # The first point cloud should be from [-0.1 to 0.1].
        # The second point cloud should be from [0.9 to 1.1].
        self.assertTrue(np.max(fused_pc.xyzs()[0, :]) >= 1.0)
        self.assertTrue(np.min(fused_pc.xyzs()[0, :]) <= 0.0)

        self.assertTrue(fused_pc.has_rgbs())
        self.assertTrue(
            np.all(fused_pc.rgbs()[:, 0] != np.array([255, 255, 255])))
        self.assertTrue(
            np.all(fused_pc.rgbs()[:, -1] != np.array([255, 255, 255])))

    def test_mix_rgb(self):
        self.pc_concat.GetInputPort("point_cloud_CiSi_0").FixValue(
            self.context, AbstractValue.Make(self.pc))
        self.pc_concat.GetInputPort("point_cloud_CiSi_1").FixValue(
            self.context, AbstractValue.Make(self.pc_no_rgbs))

        fused_pc = self.pc_concat.GetOutputPort("point_cloud_FS").Eval(
            self.context)

        self.assertEqual(fused_pc.size(), 2 * self.num_points)

        # The first point cloud should be from [-0.1 to 0.1].
        # The second point cloud should be from [0.9 to 1.1].
        self.assertTrue(np.max(fused_pc.xyzs()[0, :]) >= 1.0)
        self.assertTrue(np.min(fused_pc.xyzs()[0, :]) <= 0.0)

        self.assertTrue(fused_pc.has_rgbs())

        # We don't know in what order the two point clouds will be combined.
        rgb_first = np.all(fused_pc.rgbs()[:, 0] != np.array([255, 255, 255]))
        rgb_last = np.all(fused_pc.rgbs()[:, -1] != np.array([255, 255, 255]))
        no_rgb_first = np.all(
            fused_pc.rgbs()[:, 0] == np.array([255, 255, 255]))
        no_rgb_last = np.all(
            fused_pc.rgbs()[:, -1] == np.array([255, 255, 255]))

        self.assertTrue(
            (rgb_first and no_rgb_last) or (no_rgb_first and rgb_last))
