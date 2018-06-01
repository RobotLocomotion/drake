import pydrake.systems.sensors as mut

import numpy as np
import unittest

from pydrake.common import FindResourceOrThrow
from pydrake.multibody.rigid_body_tree import (
    AddModelInstancesFromSdfString,
    FloatingBaseType,
    RigidBodyTree,
    RigidBodyFrame,
    )
from pydrake.systems.framework import (
    InputPortDescriptor,
    OutputPort,
    Value,
    )
from pydrake.util.eigen_geometry import Isometry3

# Shorthand aliases, to reduce verbosity.
pt = mut.PixelType
pf = mut.PixelFormat

# Available image / pixel types.
pixel_types = [
    pt.kRgba8U,
    pt.kDepth32F,
    pt.kLabel16I,
]

# Convenience aliases.
image_type_aliases = [
    mut.ImageRgba8U,
    mut.ImageDepth32F,
    mut.ImageLabel16I,
]


class TestSensors(unittest.TestCase):

    def test_image_traits(self):
        # Test instantiations of ImageTraits<>.
        t = mut.ImageTraits[pt.kRgba8U]
        self.assertEqual(t.kNumChannels, 4)
        self.assertEqual(t.ChannelType, np.uint8)
        self.assertEqual(t.kPixelFormat, pf.kRgba)

        t = mut.ImageTraits[pt.kDepth32F]
        self.assertEqual(t.kNumChannels, 1)
        self.assertEqual(t.ChannelType, np.float32)
        self.assertEqual(t.kPixelFormat, pf.kDepth)

        t = mut.ImageTraits[pt.kLabel16I]
        self.assertEqual(t.kNumChannels, 1)
        self.assertEqual(t.ChannelType, np.int16)
        self.assertEqual(t.kPixelFormat, pf.kLabel)

    def test_image_types(self):
        # Test instantiations of Image<>.
        for pixel_type, image_type_alias in (
                zip(pixel_types, image_type_aliases)):
            ImageT = mut.Image[pixel_type]
            self.assertEqual(ImageT.Traits, mut.ImageTraits[pixel_type])
            self.assertEqual(ImageT, image_type_alias)

            w = 640
            h = 480
            nc = ImageT.Traits.kNumChannels
            image = ImageT(w, h)
            self.assertEqual(image.width(), w)
            self.assertEqual(image.height(), h)
            self.assertEqual(image.size(), h * w * nc)
            # N.B. Since `shape` is a custom-Python extension, it's defined as
            # a property (not a function).
            self.assertEqual(image.shape, (h, w, nc))
            self.assertEqual(image.data.shape, image.shape)
            self.assertEqual(image.data.dtype, ImageT.Traits.ChannelType)

            w /= 2
            h /= 2
            # WARNING: Resizing an image with an existing reference to
            # `image.data` will cause `image.data` + `image.mutable_data` to be
            # invalid.
            image.resize(w, h)
            self.assertEqual(image.shape, (h, w, nc))

    def test_image_data(self):
        # Test data mapping.
        for pixel_type in pixel_types:
            # Use a trivial size for ease of debugging.
            w = 8
            h = 6
            channel_default = 1
            ImageT = mut.Image[pixel_type]
            image = ImageT(w, h, channel_default)
            nc = ImageT.Traits.kNumChannels

            # Test default initialization.
            self.assertEqual(image.at(0, 0)[0], channel_default)
            self.assertTrue(np.allclose(image.data, channel_default))

            # Test pixel / channel mutation and access.
            image.at(0, 0)[0] = 2
            # - Also test named arguments.
            self.assertEqual(image.at(x=0, y=0)[0], 2)

            bad_coords = [
                # Bad X
                (-1, 0, 0),
                (100, 0, 0),
                # Bad Y
                (0, -1, 0),
                (0, 100, 0),
                # Bad Channel
                (0, 0, -100),
                (0, 0, 100),
            ]
            for x, y, c in bad_coords:
                try:
                    image.at(x, y)[c]
                    self.assertTrue(False)
                except SystemExit:
                    pass
                except IndexError:
                    pass

            # Test numpy views, access and mutation.
            image.mutable_data[:] = 3
            self.assertEqual(image.at(0, 0)[0], 3)
            self.assertTrue(np.allclose(image.data, 3))
            self.assertTrue(np.allclose(image.mutable_data, 3))

            # Ensure that each dimension of the image array is unique.
            self.assertEqual(len(set(image.shape)), 3)
            # Ensure indices match as expected. Fill each channel at each pixel
            # with unique values, and ensure that pixel / channels map
            # appropriately.
            data = image.mutable_data
            data[:] = np.arange(0, image.size()).reshape(image.shape)
            for iw in range(w):
                for ih in range(h):
                    self.assertTrue(
                        np.allclose(data[ih, iw, :], image.at(iw, ih)))

    def test_constants(self):
        # Simply ensure we can access the constants.
        values = [
            mut.InvalidDepth.kTooFar,
            mut.InvalidDepth.kTooClose,
            mut.Label.kNoBody,
            mut.Label.kFlatTerrain,
        ]
        self.assertTrue(values)

    def test_camera_info(self):
        width = 640
        height = 480
        fov_y = np.pi / 4
        focal_y = height / 2 / np.tan(fov_y / 2)
        focal_x = focal_y
        center_x = width / 2
        center_y = height / 2
        intrinsic_matrix = np.array([
            [focal_x, 0, center_x],
            [0, focal_y, center_y],
            [0, 0, 1]])

        infos = [
            mut.CameraInfo(width=width, height=height, fov_y=fov_y),
            mut.CameraInfo(
                width=width, height=height, focal_x=focal_x, focal_y=focal_y,
                center_x=center_x, center_y=center_y),
        ]

        for info in infos:
            self.assertEqual(info.width(), width)
            self.assertEqual(info.height(), height)
            self.assertEqual(info.focal_x(), focal_x)
            self.assertEqual(info.focal_y(), focal_y)
            self.assertEqual(info.center_x(), center_x)
            self.assertEqual(info.center_y(), center_y)
            self.assertTrue(
                (info.intrinsic_matrix() == intrinsic_matrix).all())

    def _check_input(self, value):
        self.assertIsInstance(value, InputPortDescriptor)

    def _check_output(self, value):
        self.assertIsInstance(value, OutputPort)

    def _check_ports(self, system):
        self._check_input(system.state_input_port())
        self._check_output(system.color_image_output_port())
        self._check_output(system.depth_image_output_port())
        self._check_output(system.label_image_output_port())
        self._check_output(system.camera_base_pose_output_port())

    def test_rgbd_camera(self):
        sdf_path = FindResourceOrThrow(
            "drake/systems/sensors/test/models/nothing.sdf")
        tree = RigidBodyTree()
        with open(sdf_path) as f:
            sdf_string = f.read()
        AddModelInstancesFromSdfString(
            sdf_string, FloatingBaseType.kFixed, None, tree)
        frame = RigidBodyFrame("rgbd camera frame", tree.FindBody("link"))
        tree.addFrame(frame)

        # Use HDTV size.
        width = 1280
        height = 720

        camera = mut.RgbdCamera(
            name="camera", tree=tree, frame=frame,
            z_near=0.5, z_far=5.0,
            fov_y=np.pi / 4, show_window=False,
            width=width, height=height)

        def check_info(camera_info):
            self.assertIsInstance(camera_info, mut.CameraInfo)
            self.assertEqual(camera_info.width(), width)
            self.assertEqual(camera_info.height(), height)

        check_info(camera.color_camera_info())
        check_info(camera.depth_camera_info())
        self.assertIsInstance(camera.color_camera_optical_pose(), Isometry3)
        self.assertIsInstance(camera.depth_camera_optical_pose(), Isometry3)
        self.assertTrue(camera.tree() is tree)
        # N.B. `RgbdCamera` copies the input frame.
        self.assertEqual(camera.frame().get_name(), frame.get_name())
        self._check_ports(camera)

        # Test discrete camera.
        period = mut.RgbdCameraDiscrete.kDefaultPeriod
        discrete = mut.RgbdCameraDiscrete(
            camera=camera, period=period, render_label_image=True)
        self.assertTrue(discrete.camera() is camera)
        self.assertTrue(discrete.mutable_camera() is camera)
        self.assertEqual(discrete.period(), period)
        self._check_ports(discrete)

        # That we can access the state as images.
        context = discrete.CreateDefaultContext()
        values = context.get_abstract_state()
        self.assertIsInstance(values.get_value(0), Value[mut.ImageRgba8U])
        self.assertIsInstance(values.get_value(1), Value[mut.ImageDepth32F])
        self.assertIsInstance(values.get_value(2), Value[mut.ImageLabel16I])
