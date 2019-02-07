from __future__ import division

import pydrake.systems.sensors as mut

import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.systems.framework import (
    AbstractValue,
    InputPort,
    OutputPort,
    Value,
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
        center_x = width / 2 - 0.5
        center_y = height / 2 - 0.5
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
            context.FixInputPort(port.get_index(), AbstractValue.Make(image))
        output = dut.AllocateOutput()
        dut.CalcOutput(context, output)
        # N.B. This Value[] is a C++ LCM object. See
        # `lcm_py_bind_cpp_serializers.h` for more information.
        self.assertIsInstance(output.get_data(0), AbstractValue)
