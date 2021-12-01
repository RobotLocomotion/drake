import pydrake.systems.sensors as mut

import gc
import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.common.test_utilities.pickle_compare import assert_pickle
from pydrake.common.value import AbstractValue, Value
from pydrake.geometry import FrameId
from pydrake.geometry.render import (
    ClippingRange,
    ColorRenderCamera,
    DepthRange,
    DepthRenderCamera,
    RenderCameraCore,
)
from pydrake.math import (
    RigidTransform,
    RollPitchYaw,
    )
from pydrake.systems.framework import (
    InputPort,
    OutputPort,
    )

# Shorthand aliases, to reduce verbosity.
pt = mut.PixelType
pf = mut.PixelFormat

# Available image / pixel types.
pixel_types = [
    pt.kRgba8U,
    pt.kDepth16U,
    pt.kDepth32F,
    pt.kLabel16I,
]

# Convenience aliases.
image_type_aliases = [
    mut.ImageRgba8U,
    mut.ImageDepth16U,
    mut.ImageDepth32F,
    mut.ImageLabel16I,
]


class TestSensors(unittest.TestCase):

    def test_image_traits(self):
        # Ensure that we test all available enums.
        self.assertSetEqual(
            set(pixel_types), set(mut.PixelType.__members__.values()))

        # Test instantiations of ImageTraits<>.
        t = mut.ImageTraits[pt.kRgba8U]
        self.assertEqual(t.kNumChannels, 4)
        self.assertEqual(t.ChannelType, np.uint8)
        self.assertEqual(t.kPixelFormat, pf.kRgba)

        t = mut.ImageTraits[pt.kDepth16U]
        self.assertEqual(t.kNumChannels, 1)
        self.assertEqual(t.ChannelType, np.uint16)
        self.assertEqual(t.kPixelFormat, pf.kDepth)

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
            image = ImageT(width=w, height=h)
            self.assertEqual(image.width(), w)
            self.assertEqual(image.height(), h)
            self.assertEqual(image.size(), h * w * nc)
            # N.B. Since `shape` is a custom-Python extension, it's defined as
            # a property (not a function).
            self.assertEqual(image.shape, (h, w, nc))
            self.assertEqual(image.data.shape, image.shape)
            self.assertEqual(image.data.dtype, ImageT.Traits.ChannelType)

            w //= 2
            h //= 2
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

            # Ensure that keep alive works by using temporary objects.

            def check_keep_alive():
                image = ImageT(w, h, channel_default)
                return (image.data, image.mutable_data)

            data, mutable_data = check_keep_alive()
            gc.collect()
            np.testing.assert_array_equal(data, channel_default)
            np.testing.assert_array_equal(mutable_data, channel_default)

    def test_camera_info(self):
        width = 640
        height = 480
        fov_y = np.pi / 4
        focal_y = height / 2 / np.tan(fov_y / 2)
        focal_x = focal_y
        center_x = width / 2 - 0.5
        center_y = height / 2 - 0.5
        intrinsic_matrix = np.array([
            [focal_x, 0, center_x],
            [0, focal_y, center_y],
            [0, 0, 1]])

        infos = [
            mut.CameraInfo(width=width, height=height, fov_y=fov_y),
            mut.CameraInfo(
                width=width, height=height, intrinsic_matrix=intrinsic_matrix),
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
            assert_pickle(self, info, mut.CameraInfo.intrinsic_matrix)

    def _check_input(self, value):
        self.assertIsInstance(value, InputPort)

    def _check_output(self, value):
        self.assertIsInstance(value, OutputPort)

    def test_image_to_lcm_image_array_t(self):
        # Test nominal constructor.
        dut = mut.ImageToLcmImageArrayT(
            color_frame_name="color", depth_frame_name="depth",
            label_frame_name="label", do_compress=False)
        for port in (
                dut.color_image_input_port(), dut.depth_image_input_port(),
                dut.label_image_input_port()):
            self._check_input(port)
        self._check_output(dut.image_array_t_msg_output_port())

        # Test custom constructor, test functionality (up to getting abstract
        # value).
        dut = mut.ImageToLcmImageArrayT(do_compress=False)
        # Declare ports.
        for pixel_type in pixel_types:
            name = str(pixel_type)
            dut.DeclareImageInputPort[pixel_type](name=name)
        context = dut.CreateDefaultContext()
        for pixel_type in pixel_types:
            name = str(pixel_type)
            port = dut.GetInputPort(name)
            self._check_input(port)
            image = mut.Image[pixel_type](width=1, height=1)
            port.FixValue(context, image)
        output = dut.AllocateOutput()
        dut.CalcOutput(context, output)
        # N.B. This Value[] is a C++ LCM object. See
        # `lcm_py_bind_cpp_serializers.h` for more information.
        self.assertIsInstance(output.get_data(0), AbstractValue)

    def test_rgbd_sensor(self):
        def check_ports(system):
            self.assertIsInstance(system.query_object_input_port(), InputPort)
            self.assertIsInstance(system.color_image_output_port(), OutputPort)
            self.assertIsInstance(system.depth_image_32F_output_port(),
                                  OutputPort)
            self.assertIsInstance(system.depth_image_16U_output_port(),
                                  OutputPort)
            self.assertIsInstance(system.label_image_output_port(), OutputPort)
            self.assertIsInstance(system.body_pose_in_world_output_port(),
                                  OutputPort)

        # Use HDTV size.
        width = 1280
        height = 720

        # There are *two* variants of the constructor for each camera
        # representation: one with color and depth explicitly specified and one
        # with only depth. We enumerate all four here.

        def construct(parent_id, X_PB):
            color_camera = ColorRenderCamera(
                RenderCameraCore(
                    "renderer",
                    mut.CameraInfo(width, height, np.pi/6),
                    ClippingRange(0.1, 6.0),
                    RigidTransform()
                ), False)
            depth_camera = DepthRenderCamera(color_camera.core(),
                                             DepthRange(0.1, 5.5))
            return mut.RgbdSensor(parent_id=parent_id, X_PB=X_PB,
                                  color_camera=color_camera,
                                  depth_camera=depth_camera)

        def construct_single(parent_id, X_PB):
            depth_camera = DepthRenderCamera(
                RenderCameraCore(
                    "renderer",
                    mut.CameraInfo(width, height, np.pi/6),
                    ClippingRange(0.1, 6.0),
                    RigidTransform()
                ),
                DepthRange(0.1, 5.5))
            return mut.RgbdSensor(parent_id=parent_id, X_PB=X_PB,
                                  depth_camera=depth_camera)

        # Put it at the origin.
        X_WB = RigidTransform()
        # This id would fail if we tried to render; no such id exists.
        parent_id = FrameId.get_new_id()

        def check_info(camera_info):
            self.assertIsInstance(camera_info, mut.CameraInfo)
            self.assertEqual(camera_info.width(), width)
            self.assertEqual(camera_info.height(), height)

        for constructor in [construct, construct_single]:
            sensor = constructor(parent_id, X_WB)
            check_info(sensor.color_camera_info())
            check_info(sensor.depth_camera_info())
            self.assertIsInstance(sensor.X_BC(),
                                  RigidTransform)
            self.assertIsInstance(sensor.X_BD(),
                                  RigidTransform)
            self.assertEqual(sensor.parent_frame_id(), parent_id)
            check_ports(sensor)

        # Test discrete camera. We'll simply use the last sensor constructed.

        period = mut.RgbdSensorDiscrete.kDefaultPeriod
        discrete = mut.RgbdSensorDiscrete(
            sensor=sensor, period=period, render_label_image=True)
        self.assertTrue(discrete.sensor() is sensor)
        self.assertEqual(discrete.period(), period)
        check_ports(discrete)

        # That we can access the state as images.
        context = discrete.CreateDefaultContext()
        values = context.get_abstract_state()
        self.assertIsInstance(values.get_value(0),
                              Value[mut.ImageRgba8U])
        self.assertIsInstance(values.get_value(1),
                              Value[mut.ImageDepth32F])
        self.assertIsInstance(values.get_value(2),
                              Value[mut.ImageDepth16U])
        self.assertIsInstance(values.get_value(3),
                              Value[mut.ImageLabel16I])
