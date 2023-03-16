import pydrake.perception as mut

import unittest

import numpy as np

from pydrake.common.value import AbstractValue, Value
from pydrake.systems.sensors import CameraInfo, PixelType
from pydrake.systems.framework import InputPort, OutputPort


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
        # TODO(jwnimmer-tri) Ideally, these would map back to the bitwise
        # constants, but for now this works; the ctor accepts bare ints.
        self.assertEqual(repr(fields_none), "Fields(base_fields=0)")
        self.assertEqual(repr(fields), "Fields(base_fields=2)")
        repr_and_back = eval(repr(fields), dict(Fields=mut.Fields))
        self.assertEqual(repr_and_back.base_fields(), enum.kXYZs)

    def check_array(self, x, dtype_expected, shape_expected):
        x = np.asarray(x)
        self.assertEqual(x.dtype, dtype_expected)
        self.assertTupleEqual(x.shape, shape_expected)

    def test_point_cloud_api(self):
        self.assertEqual(mut.PointCloud.T, np.float32)
        self.assertEqual(mut.PointCloud.C, np.uint8)
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
        # - Additional types are tested via simple existence checks.
        all_fields = mut.Fields(
            mut.BaseField.kXYZs | mut.BaseField.kNormals | mut.BaseField.kRGBs)
        count = 1
        pc = mut.PointCloud(new_size=count, fields=all_fields)
        self.check_array(pc.mutable_xyzs(), np.float32, (3, count))
        self.check_array(pc.mutable_normals(), np.float32, (3, count))
        pc.normals()
        pc.mutable_normal(i=0)
        pc.normal(i=0)
        pc.FlipNormalsTowardPoint(p_CP=[0, 0, 1])
        self.check_array(pc.mutable_rgbs(), np.uint8, (3, count))
        pc.rgbs()
        pc.mutable_rgb(i=0)
        pc.rgb(i=0)
        # Test morphing fields after PointCloud creation.
        rgb_pc = mut.PointCloud(new_size=count,
                                fields=mut.Fields(mut.BaseField.kRGBs))
        all_fields_pc = mut.PointCloud(new_size=count, fields=all_fields)
        test_rgbs = [[1, 2, 3]]
        all_fields_pc.mutable_rgbs().T[:] = test_rgbs
        self.assertFalse((rgb_pc.rgbs().T == test_rgbs).all())
        self.assertFalse(rgb_pc.has_xyzs())
        rgb_pc = all_fields_pc
        self.assertTrue((rgb_pc.rgbs().T == test_rgbs).all())
        self.assertTrue(rgb_pc.has_xyzs())
        self.assertTrue(rgb_pc.has_normals())
        rgb_pc.SetFields(new_fields=mut.Fields(mut.BaseField.kNormals),
                         skip_initialize=False)
        self.assertFalse(rgb_pc.has_rgbs())
        self.assertTrue(rgb_pc.has_normals())
        # - Check for none.
        with self.assertRaises(RuntimeError) as ex:
            mut.PointCloud(new_size=0, fields=mut.Fields(mut.BaseField.kNone))
        # Test Systems' value registration.
        self.assertIsInstance(AbstractValue.Make(pc), Value[mut.PointCloud])

        pc = mut.PointCloud(new_size=2, fields=mut.Fields(mut.BaseField.kXYZs))
        test_xyzs = [[1., 2., 3.], [4., 5., 6.]]
        pc.mutable_xyzs().T[:] = test_xyzs
        crop = pc.Crop(lower_xyz=[3, 4, 5], upper_xyz=[5, 6, 7])
        self.assertEqual(crop.size(), 1)

        pc_merged_1 = mut.Concatenate(clouds=[pc, pc_new])
        pc_merged_2 = mut.Concatenate(clouds=[pc, pc_new])
        self.assertEqual(pc_merged_1.size(), pc.size() + pc_new.size())
        self.assertEqual(pc_merged_2.size(), pc.size() + pc_new.size())

        pc_downsampled_1 = pc_merged_1.VoxelizedDownSample(voxel_size=2.0)
        self.assertIsInstance(pc_downsampled_1, mut.PointCloud)

        pc_downsampled_2 = pc_merged_2.VoxelizedDownSample(
            voxel_size=2.0, parallelize=False)
        self.assertIsInstance(pc_downsampled_2, mut.PointCloud)

        self.assertFalse(pc_merged_1.has_normals())
        pc_merged_1.EstimateNormals(radius=1, num_closest=50)
        self.assertTrue(pc_merged_1.has_normals())

        self.assertFalse(pc_merged_2.has_normals())
        pc_merged_2.EstimateNormals(
            radius=1, num_closest=50, parallelize=False)
        self.assertTrue(pc_merged_2.has_normals())

    def test_depth_image_to_point_cloud_api(self):
        camera_info = CameraInfo(width=640, height=480, fov_y=np.pi / 4)
        dut = mut.DepthImageToPointCloud(camera_info=camera_info)
        self.assertIsInstance(dut.depth_image_input_port(), InputPort)
        self.assertIsInstance(dut.color_image_input_port(), InputPort)
        self.assertIsInstance(dut.camera_pose_input_port(), InputPort)
        self.assertIsInstance(dut.point_cloud_output_port(), OutputPort)
        dut = mut.DepthImageToPointCloud(
            camera_info=camera_info,
            pixel_type=PixelType.kDepth16U,
            scale=0.001,
            fields=mut.BaseField.kXYZs | mut.BaseField.kRGBs)

    def test_point_cloud_to_lcm(self):
        dut = mut.PointCloudToLcm(frame_name="world")
        dut.get_input_port()
        dut.get_output_port()
