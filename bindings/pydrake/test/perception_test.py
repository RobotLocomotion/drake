import pydrake.perception as mut

import unittest

import numpy as np

from pydrake.systems.sensors import CameraInfo, ImageDepth32F
from pydrake.systems.framework import (
    AbstractValue, InputPort, OutputPort, Value)


class TestPerception(unittest.TestCase):
    def test_pc_flags_api(self):
        # BaseField
        enum = mut.BaseField
        enum.kNone
        enum.kXYZs
        # Fields
        fields = mut.Fields(base_fields=enum.kXYZs)
        self.assertEqual(fields.base_fields(), enum.kXYZs)
        self.assertTrue(fields.has_base_fields())
        self.assertEqual(fields, fields)
        fields_none = mut.Fields(enum.kNone)
        self.assertNotEqual(fields, fields_none)

    def test_point_cloud_api(self):
        self.assertEqual(mut.PointCloud.T, np.float32)
        self.assertEqual(mut.PointCloud.D, np.float32)
        self.assertTrue(mut.PointCloud.IsDefaultValue(
            value=mut.PointCloud.kDefaultValue))
        self.assertTrue(mut.PointCloud.IsInvalidValue(
            value=mut.PointCloud.kDefaultValue))

        fields = mut.Fields(mut.BaseField.kXYZs)
        pc = mut.PointCloud(new_size=0, fields=fields)
        self.assertEqual(pc.fields(), fields)
        self.assertEqual(pc.size(), 0)
        pc.resize(new_size=2)
        self.assertEqual(pc.size(), 2)
        self.assertTrue(pc.has_xyzs())
        self.assertEqual(pc.xyzs().shape, (3, 2))
        # Test reference semantics with NumPy + Pybind11.
        test_xyzs = [[1., 2., 3.], [4., 5., 6.]]
        pc.mutable_xyzs().T[:] = test_xyzs
        np.testing.assert_equal(pc.xyzs().T, test_xyzs)
        test_xyz = [10., 20., 30.]
        pc.mutable_xyz(i=0)[:] = test_xyz
        np.testing.assert_equal(pc.xyz(i=0), test_xyz)
        np.testing.assert_equal(pc.xyzs().T[0], test_xyz)
        pc_new = mut.PointCloud()
        pc_new.SetFrom(other=pc)
        np.testing.assert_equal(pc.xyzs(), pc_new.xyzs())
        # Test Systems' value registration.
        self.assertIsInstance(AbstractValue.Make(pc), Value[mut.PointCloud])

    def test_depth_image_to_point_cloud_api(self):
        camera_info = CameraInfo(width=640, height=480, fov_y=np.pi / 4)
        dut = mut.DepthImageToPointCloud(camera_info=camera_info)
        self.assertIsInstance(dut.depth_image_input_port(), InputPort)
        self.assertIsInstance(dut.point_cloud_output_port(), OutputPort)
