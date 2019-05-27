# -*- coding: utf-8 -*-

import unittest
import numpy as np

from pydrake.math import RigidTransform
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import AbstractValue, DiagramBuilder
from pydrake.systems.point_cloud_synthesis import PointCloudSynthesis

import pydrake.perception as mut


class TestPointCloudSynthesis(unittest.TestCase):
    def setUp(self):
        builder = DiagramBuilder()

        X_WP_0 = RigidTransform.Identity()
        X_WP_1 = RigidTransform.Identity()
        X_WP_1.set_translation([1.0, 0, 0])

        id_list = ["0", "1"]

        self.pc_synth = builder.AddSystem(PointCloudSynthesis(id_list))

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
            self.pc_synth, simulator.get_mutable_context())

        self.context.FixInputPort(
            self.pc_synth.GetInputPort("X_WP_0").get_index(),
            AbstractValue.Make(X_WP_0))
        self.context.FixInputPort(
            self.pc_synth.GetInputPort("X_WP_1").get_index(),
            AbstractValue.Make(X_WP_1))

    def test_no_rgb(self):
        self.context.FixInputPort(
            self.pc_synth.GetInputPort("point_cloud_P_0").get_index(),
            AbstractValue.Make(self.pc_no_rgbs))
        self.context.FixInputPort(
            self.pc_synth.GetInputPort("point_cloud_P_1").get_index(),
            AbstractValue.Make(self.pc_no_rgbs))

        fused_pc = self.pc_synth.GetOutputPort("combined_point_cloud_W").Eval(
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
            self.pc_synth.GetInputPort("point_cloud_P_0").get_index(),
            AbstractValue.Make(self.pc))
        self.context.FixInputPort(
            self.pc_synth.GetInputPort("point_cloud_P_1").get_index(),
            AbstractValue.Make(self.pc))

        fused_pc = self.pc_synth.GetOutputPort("combined_point_cloud_W").Eval(
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
            self.pc_synth.GetInputPort("point_cloud_P_0").get_index(),
            AbstractValue.Make(self.pc))
        self.context.FixInputPort(
            self.pc_synth.GetInputPort("point_cloud_P_1").get_index(),
            AbstractValue.Make(self.pc_no_rgbs))

        fused_pc = self.pc_synth.GetOutputPort("combined_point_cloud_W").Eval(
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
