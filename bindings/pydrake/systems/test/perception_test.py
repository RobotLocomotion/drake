# -*- coding: utf-8 -*-

import unittest
import numpy as np

from pydrake.math import RigidTransform
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import AbstractValue, DiagramBuilder
from pydrake.systems.perception import (
    PointCloudConcatenation, _ConcatenatePointClouds)

import pydrake.perception as mut


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

        self.pc = mut.PointCloud(
            self.num_points,
            mut.Fields(mut.BaseField.kXYZs | mut.BaseField.kRGBs))
        self.pc.mutable_xyzs()[:] = xyzs
        self.pc.mutable_rgbs()[:] = rgbs

        self.pc_no_rgbs = mut.PointCloud(
            self.num_points, mut.Fields(mut.BaseField.kXYZs))
        self.pc_no_rgbs.mutable_xyzs()[:] = xyzs

        diagram = builder.Build()

        simulator = Simulator(diagram)

        self.context = diagram.GetMutableSubsystemContext(
            self.pc_concat, simulator.get_mutable_context())

        self.context.FixInputPort(
            self.pc_concat.GetInputPort("X_WP_0").get_index(),
            AbstractValue.Make(X_WP_0))
        self.context.FixInputPort(
            self.pc_concat.GetInputPort("X_WP_1").get_index(),
            AbstractValue.Make(X_WP_1))

    def test_no_rgb(self):
        self.context.FixInputPort(
            self.pc_concat.GetInputPort("point_cloud_P_0").get_index(),
            AbstractValue.Make(self.pc_no_rgbs))
        self.context.FixInputPort(
            self.pc_concat.GetInputPort("point_cloud_P_1").get_index(),
            AbstractValue.Make(self.pc_no_rgbs))

        fused_pc = self.pc_concat.GetOutputPort("combined_point_cloud_W").Eval(
            self.context)

        self.assertEqual(fused_pc.size(), 2 * self.num_points)

        # The first point cloud should be from [-0.1 to 0.1].
        # Tthe second point cloud should be from [0.9 to 1.1].
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
        self.context.FixInputPort(
            self.pc_concat.GetInputPort("point_cloud_P_0").get_index(),
            AbstractValue.Make(self.pc))
        self.context.FixInputPort(
            self.pc_concat.GetInputPort("point_cloud_P_1").get_index(),
            AbstractValue.Make(self.pc))

        fused_pc = self.pc_concat.GetOutputPort("combined_point_cloud_W").Eval(
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
        self.context.FixInputPort(
            self.pc_concat.GetInputPort("point_cloud_P_0").get_index(),
            AbstractValue.Make(self.pc))
        self.context.FixInputPort(
            self.pc_concat.GetInputPort("point_cloud_P_1").get_index(),
            AbstractValue.Make(self.pc_no_rgbs))

        fused_pc = self.pc_concat.GetOutputPort("combined_point_cloud_W").Eval(
            self.context)

        self.assertEqual(fused_pc.size(), 2 * self.num_points)

        # The first point cloud should be from [-0.1 to 0.1].
        # The second point cloud should be from [0.9 to 1.1].
        self.assertTrue(np.max(fused_pc.xyzs()[0, :]) >= 1.0)
        self.assertTrue(np.min(fused_pc.xyzs()[0, :]) <= 0.0)

        self.assertTrue(fused_pc.has_rgbs())

        # We don't know what order the two point clouds will be combined.
        rgb_first = np.all(fused_pc.rgbs()[:, 0] != np.array([255, 255, 255]))
        rgb_last = np.all(fused_pc.rgbs()[:, -1] != np.array([255, 255, 255]))
        no_rgb_first = np.all(
            fused_pc.rgbs()[:, 0] == np.array([255, 255, 255]))
        no_rgb_last = np.all(
            fused_pc.rgbs()[:, -1] == np.array([255, 255, 255]))

        self.assertTrue(
            (rgb_first and no_rgb_last) or (no_rgb_first and rgb_last))
