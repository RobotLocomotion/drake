import pydrake.systems.sensors as mut

import copy
import gc
import tempfile
import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.common.test_utilities.pickle_compare import assert_pickle
from pydrake.common.value import AbstractValue, Value
from pydrake.geometry import (
    ClippingRange,
    ColorRenderCamera,
    DepthRange,
    DepthRenderCamera,
    FrameId,
    RenderCameraCore,
)
from pydrake.lcm import DrakeLcm
from pydrake.math import (
    RigidTransform,
    RollPitchYaw,
)
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
)
from pydrake.systems.framework import (
    DiagramBuilder,
    InputPort,
    OutputPort,
    )
from pydrake.systems.lcm import LcmBuses, _Serializer_
from drake import (
    lcmt_image,
    lcmt_image_array,
)

# Shorthand aliases, to reduce verbosity.
pt = mut.PixelType
pf = mut.PixelFormat

# Available image / pixel types.
pixel_types = [
    pt.kRgba8U,
    pt.kRgb8U,
    pt.kBgra8U,
    pt.kBgr8U,
    pt.kDepth16U,
    pt.kDepth32F,
    pt.kLabel16I,
    pt.kGrey8U,
]

# Convenience aliases.
image_type_aliases = [
    mut.ImageRgba8U,
    mut.ImageRgb8U,
    mut.ImageBgra8U,
    mut.ImageBgr8U,
    mut.ImageDepth16U,
    mut.ImageDepth32F,
    mut.ImageLabel16I,
    mut.ImageGrey8U,
]


class TestSensors(unittest.TestCase):

    def test_image_traits(self):
        # Ensure that we test all available enums.
        self.assertSetEqual(
            set(pixel_types), set(mut.PixelType.__members__.values()))

        # Spot-check specific instantiations of ImageTraits<>.
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

        # Smoke test all instantiations of ImageTraits<>.
        for pixel_type in pixel_types:
            t = mut.ImageTraits[pixel_type]
            self.assertGreaterEqual(t.kNumChannels, 1)
            self.assertIsNotNone(t.ChannelType)
            self.assertIn(t.kPixelFormat, mut.PixelFormat.__members__.values())

        # Smoke test the pixel scalars.
        mut.PixelScalar.k8U
        mut.PixelScalar.k16I
        mut.PixelScalar.k16U
        mut.PixelScalar.k32F

    def test_image_types(self):
        # Test instantiations of Image<>.
        for pixel_type, image_type_alias in (
                zip(pixel_types, image_type_aliases)):
            ImageT = mut.Image[pixel_type]
            self.assertEqual(ImageT.Traits, mut.ImageTraits[pixel_type])
            self.assertEqual(ImageT, image_type_alias)
            ImageT()

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

    def test_depth_image_conversion(self):
        foo = mut.ImageDepth32F(width=3, height=4)
        bar = mut.ImageDepth16U()
        mut.ConvertDepth32FTo16U(input=foo, output=bar)
        self.assertEqual(bar.width(), 3)
        self.assertEqual(bar.height(), 4)

        foo = mut.ImageDepth16U(width=3, height=4)
        bar = mut.ImageDepth32F()
        mut.ConvertDepth16UTo32F(input=foo, output=bar)
        self.assertEqual(bar.width(), 3)
        self.assertEqual(bar.height(), 4)

    def test_camera_config(self):
        mut.CameraConfig()
        config = mut.CameraConfig(
            width=124, focal=mut.CameraConfig.FocalLength(x=10, y=20))
        self.assertEqual(config.width, 124)
        self.assertIn("width", repr(config))
        copy.copy(config)
        self.assertEqual(config.focal_x(), 10)
        self.assertEqual(config.focal_y(), 20)
        config.principal_point()
        (color, depth) = config.MakeCameras()
        self.assertIsInstance(color, ColorRenderCamera)
        self.assertIsInstance(depth, DepthRenderCamera)

        fov = mut.CameraConfig.FovDegrees(x=10, y=20)
        self.assertIn("x=10", repr(fov))
        copy.copy(fov)

        focal = mut.CameraConfig.FocalLength(x=10, y=20)
        self.assertIn("x=10", repr(focal))
        copy.copy(focal)

        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        system_count = len(builder.GetSystems())
        lcm = DrakeLcm()
        mut.ApplyCameraConfig(config=config, plant=plant, builder=builder,
                              scene_graph=scene_graph, lcm=lcm)
        # Systems have been added.
        self.assertGreater(len(builder.GetSystems()), system_count)

    def test_camera_config_lcm_buses(self):
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        system_count = len(builder.GetSystems())

        # We'll call the Apply function using lcm_buses= instead of lcm=.
        lcm_buses = LcmBuses()
        lcm_buses.Add("fancy", DrakeLcm())
        config = mut.CameraConfig(lcm_bus="fancy")
        mut.ApplyCameraConfig(config=config, builder=builder,
                              lcm_buses=lcm_buses)

        # Check that systems were added.
        self.assertGreater(len(builder.GetSystems()), system_count)

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
            self.assertIsInstance(info.fov_x(), float)
            self.assertIsInstance(info.fov_y(), float)
            self.assertTrue(
                (info.intrinsic_matrix() == intrinsic_matrix).all())
            assert_pickle(self, info, mut.CameraInfo.intrinsic_matrix)

    def _check_input(self, value):
        self.assertIsInstance(value, InputPort)

    def _check_output(self, value):
        self.assertIsInstance(value, OutputPort)

    def test_image_to_lcm_image_array_basic(self):
        """Tests the nominal constructor."""
        dut = mut.ImageToLcmImageArrayT(
            color_frame_name="color", depth_frame_name="depth",
            label_frame_name="label", do_compress=False)
        for port in (
                dut.color_image_input_port(), dut.depth_image_input_port(),
                dut.label_image_input_port()):
            self._check_input(port)
        for port in (
                dut.image_array_t_msg_output_port(),):
            self._check_output(port)

    def test_image_to_lcm_image_array_custom(self):
        """Tests the custom constructor and runtime functionality."""
        # Declare ports using the custom constructor.
        dut = mut.ImageToLcmImageArrayT(do_compress=False)
        for pixel_type in pixel_types:
            name = str(pixel_type)
            dut.DeclareImageInputPort[pixel_type](name=name)

        # Populate the input images.
        context = dut.CreateDefaultContext()
        for pixel_type in pixel_types:
            name = str(pixel_type)
            port = dut.GetInputPort(name)
            self._check_input(port)
            image = mut.Image[pixel_type](width=1, height=1)
            port.FixValue(context, image)

        # Compute the C++ message as a Value<drake::lcmt_image_array>.
        output = dut.AllocateOutput()
        dut.CalcOutput(context, output)
        cxx_message = output.get_data(0)

        # We can't access that C++ message from Python (messages are not bound
        # into the Value[] template class), so to inspect it we'll need to
        # serialize down into raw bytes and then back into a Python message.
        serializer = _Serializer_[lcmt_image_array]()
        message = lcmt_image_array.decode(serializer.Serialize(cxx_message))

        # Inspect the message for correctness.
        self.assertEqual(message.num_images, len(pixel_types))
        for i, image in enumerate(message.images):
            pixel_type = pixel_types[i]
            with self.subTest(pixel_type=pixel_type):
                self.assertEqual(image.width, 1)
                self.assertEqual(image.height, 1)
                expected_format = {
                    pt.kRgba8U: lcmt_image.PIXEL_FORMAT_RGBA,
                    pt.kRgb8U: lcmt_image.PIXEL_FORMAT_RGB,
                    pt.kBgra8U: lcmt_image.PIXEL_FORMAT_BGRA,
                    pt.kBgr8U: lcmt_image.PIXEL_FORMAT_BGR,
                    pt.kDepth16U: lcmt_image.PIXEL_FORMAT_DEPTH,
                    pt.kDepth32F: lcmt_image.PIXEL_FORMAT_DEPTH,
                    pt.kLabel16I: lcmt_image.PIXEL_FORMAT_LABEL,
                    pt.kGrey8U: lcmt_image.PIXEL_FORMAT_GRAY,
                }[pixel_type]
                self.assertEqual(image.pixel_format, expected_format)

    def test_lcm_image_array_to_images_basic(self):
        """Tests all API calls as well as runtime functionality."""
        dut = mut.LcmImageArrayToImages()
        for port in (
                dut.image_array_t_input_port(),):
            self._check_input(port)
        for port in (
                dut.color_image_output_port(),
                dut.depth_image_output_port()):
            self._check_output(port)

        # Create a one-pixel lcmt_image message.
        image_message = lcmt_image()
        image_message.pixel_format = lcmt_image.PIXEL_FORMAT_RGBA
        image_message.channel_type = lcmt_image.CHANNEL_TYPE_UINT8
        image_message.width = 1
        image_message.height = 1
        image_message.row_stride = 4
        image_message.size = 4
        image_message.data = [0] * 4

        # Wrap the single image message into an image_array message.
        array_message = lcmt_image_array()
        array_message.num_images = 1
        array_message.images = [image_message]

        # Copy the message into the Context. This is a bit tricky because the
        # input port is Value<drake::lcmt_image_array> which is not bound into
        # the Value[] class template, so we need to encode/decode to get there.
        context = dut.CreateDefaultContext()
        serializer = _Serializer_[lcmt_image_array]()
        cxx_message = serializer.CreateDefaultValue()
        serializer.Deserialize(array_message.encode(), cxx_message)
        dut.image_array_t_input_port().FixValue(context, cxx_message)

        # Extract the message's color image using the dut.
        image = dut.color_image_output_port().Eval(context)
        self.assertEqual(image.width(), 1)
        self.assertEqual(image.height(), 1)

    @staticmethod
    def _make_render_camera_core(*, width=640, height=480):
        return RenderCameraCore(
            "renderer",
            mut.CameraInfo(width, height, np.pi/6),
            ClippingRange(0.1, 6.0),
            RigidTransform())

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
            self.assertIsInstance(system.image_time_output_port(), OutputPort)

        # Use HDTV size.
        width = 1280
        height = 720

        # There are *two* variants of the constructor for each camera
        # representation: one with color and depth explicitly specified and one
        # with only depth. We enumerate all four here.

        def construct(parent_id, X_PB):
            color_camera = ColorRenderCamera(
                self._make_render_camera_core(width=width, height=height),
                False)
            depth_camera = DepthRenderCamera(color_camera.core(),
                                             DepthRange(0.1, 5.5))
            return mut.RgbdSensor(parent_id=parent_id, X_PB=X_PB,
                                  color_camera=color_camera,
                                  depth_camera=depth_camera)

        def construct_single(parent_id, X_PB):
            depth_camera = DepthRenderCamera(
                self._make_render_camera_core(width=width, height=height),
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

    def test_rgbd_sensor_async(self):
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        camera_core = self._make_render_camera_core()
        color_camera = ColorRenderCamera(camera_core)
        depth_camera = DepthRenderCamera(camera_core, DepthRange(0.1, 5.5))
        dut = mut.RgbdSensorAsync(scene_graph=scene_graph,
                                  parent_id=FrameId.get_new_id(),
                                  X_PB=RigidTransform(),
                                  fps=1.0,
                                  capture_offset=0.1,
                                  output_delay=0.01,
                                  color_camera=color_camera,
                                  depth_camera=depth_camera,
                                  render_label_image=True)
        dut.parent_id()
        dut.X_PB()
        dut.fps()
        dut.capture_offset()
        dut.output_delay()
        dut.color_camera()
        dut.depth_camera()
        dut.color_image_output_port()
        dut.depth_image_32F_output_port()
        dut.depth_image_16U_output_port()
        dut.label_image_output_port()
        dut.body_pose_in_world_output_port()
        dut.image_time_output_port()

    def test_image_file_format(self):
        mut.ImageFileFormat.kJpeg
        mut.ImageFileFormat.kPng
        mut.ImageFileFormat.kTiff

    def test_image_io_metadata(self):
        dut = mut.ImageIo.Metadata(width=640)
        self.assertEqual(dut.width, 640)
        self.assertIn("width=640", repr(dut))

    def test_image_io_using_buffer(self):
        orig_image = mut.ImageRgba8U(6, 4)

        format = mut.ImageFileFormat.kPng
        dut = mut.ImageIo()
        data = dut.Save(image=orig_image, format=format)
        self.assertIsInstance(data, bytes)
        self.assertGreater(len(data), 0)

        meta = dut.LoadMetadata(buffer=data)
        self.assertEqual((meta.width, meta.height), (6, 4))

        new_image = dut.Load(buffer=data, format=format)
        self.assertEqual((new_image.width(), new_image.height()), (6, 4))

    def test_image_io_using_file(self):
        orig_image = mut.ImageRgba8U(6, 4)

        with tempfile.TemporaryDirectory() as temp:
            path = f"{temp}/test_image_io_using_file.png"

            dut = mut.ImageIo()
            dut.Save(image=orig_image, path=path, format=None)

            meta = dut.LoadMetadata(path=path)
            self.assertEqual((meta.width, meta.height), (6, 4))

            new_image = dut.Load(path=path, format=mut.ImageFileFormat.kPng)
            self.assertEqual((new_image.width(), new_image.height()), (6, 4))

    def test_image_writer(self):
        writer = mut.ImageWriter()
        input_port = writer.DeclareImageInputPort(
            pixel_type=mut.PixelType.kRgba8U,
            port_name="color",
            file_name_format="/tmp/{port_name}-{time_usec}",
            publish_period=0.125,
            start_time=0.0)
        self.assertIsNotNone(input_port)
