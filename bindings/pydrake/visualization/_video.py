import functools
import matplotlib as mpl
import numpy as np

from pydrake.common.value import AbstractValue
from pydrake.geometry import SceneGraph
from pydrake.geometry.render import (
    ClippingRange,
    DepthRange,
    DepthRenderCamera,
    MakeRenderEngineVtk,
    RenderCameraCore,
    RenderEngineVtkParams,
    RenderLabel,
)
from pydrake.math import RigidTransform
from pydrake.systems.framework import (
    EventStatus,
    LeafSystem,
)
from pydrake.systems.sensors import (
    CameraInfo,
    ImageDepth32F,
    ImageLabel16I,
    ImageRgba8U,
    RgbdSensor,
)


# TODO(jwnimmer-tri) Move this system to C++ so everyone can use it.
class ColorizeDepthImage(LeafSystem):
    """Converts a depth image to a color image.

    .. pydrake_system::

        name: ColorizeDepthImage
        input_ports:
        - depth_image_32f
        output_ports:
        - color_image
    """

    def __init__(self):
        LeafSystem.__init__(self)
        self._depth32_input = self.DeclareAbstractInputPort(
            name="depth_image_32f",
            model_value=AbstractValue.Make(ImageDepth32F(0, 0)))
        self._color_output = self.DeclareAbstractOutputPort(
            "color_image",
            alloc=lambda: AbstractValue.Make(ImageRgba8U(0, 0)),
            calc=self._calc_output)
        self.invalid_color = (100, 0, 0, 255)

    def _calc_output(self, context, output):
        """Implements the color_image output function."""
        depth = self._depth32_input.Eval(context)
        color = output.get_mutable_value()
        self._colorize_depth_image(depth, color)

    def _colorize_depth_image(self, depth, color):
        """Colorizes an ImageDepth32F into an ImageRgba8U."""
        if not all([color.width() == depth.width(),
                    color.height() == depth.height()]):
            color.resize(depth.width(), depth.height())
        depth_array = depth.data.squeeze(2)
        color.mutable_data[:] = self._colorize_depth_array(depth_array)

    def _colorize_depth_array(self, depth):
        """Colorizes an np.array of depths into an np.array of rgba."""
        assert depth.dtype == np.float32
        h, w = depth.shape
        invalid = (depth <= 0) | ~np.isfinite(depth)
        scale_min = np.min(depth[~invalid])
        scale_max = np.max(depth[~invalid])
        # Normalize.
        depth_scaled = (depth - scale_min) / (scale_max - scale_min)
        # Convert to rgb.
        depth_rgb = np.tile(depth_scaled.reshape((h, w, 1)), (1, 1, 3))
        # Invert coloring (white is up front).
        depth_rgb = 1 - depth_rgb
        # Switch to uint8 and add alpha.
        depth_rgb = (depth_rgb * 255).astype(np.uint8)
        alpha = np.full((h, w, 1), 255, dtype=np.uint8)
        color = np.concatenate((depth_rgb, alpha), axis=2)
        color[invalid] = np.array(self.invalid_color)
        return color


# TODO(jwnimmer-tri) Move this system to C++ so everyone can use it.
class ColorizeLabelImage(LeafSystem):
    """Converts a label image to a color image.

    .. pydrake_system::

        name: ColorizeLabelImage
        input_ports:
        - label_image
        output_ports:
        - color_image
    """

    def __init__(self):
        LeafSystem.__init__(self)
        self._label_input = self.DeclareAbstractInputPort(
            name="label_image",
            model_value=AbstractValue.Make(ImageLabel16I(0, 0)))
        self._color_output = self.DeclareAbstractOutputPort(
            "color_image",
            alloc=lambda: AbstractValue.Make(ImageRgba8U(0, 0)),
            calc=self._calc_output)
        self._palette = self._make_palette()

    def _calc_output(self, context, output):
        """Implements the color_image output function."""
        label = self._label_input.Eval(context)
        color = output.get_mutable_value()
        self._colorize_label_image(label, color)

    def _colorize_label_image(self, label, color):
        """Colorizes an ImageLabel16I into an ImageRgba8U."""
        if not all([color.width() == label.width(),
                    color.height() == label.height()]):
            color.resize(label.width(), label.height())
        label_array = label.data.squeeze(2)
        color.mutable_data[:] = self._colorize_label_array(label_array)

    def _colorize_label_array(self, label):
        """Colorizes an np.array of labels into an np.array of rgba."""
        assert label.dtype == np.int16
        background = np.zeros(label.shape[:2], dtype=bool)
        reserved_labels = [
            RenderLabel.kDoNotRender,
            RenderLabel.kDontCare,
            RenderLabel.kEmpty,
            RenderLabel.kUnspecified,
        ]
        for x in reserved_labels:
            background |= label == int(x)
        color = self._palette[label.squeeze() % len(self._palette)]
        color[background] = [0, 0, 0, 0]
        return color

    @staticmethod
    def _make_palette():
        """Creates the rgba label palette."""
        values = []
        values.extend(mpl.colors.TABLEAU_COLORS.values())
        values.extend(mpl.colors.BASE_COLORS.values())
        cc = mpl.colors.ColorConverter()
        rgba_float = np.array([cc.to_rgba(x) for x in values])
        return (rgba_float*255).astype(np.uint8)


# TODO(jwnimmer-tri) Move this system to C++ so everyone can use it.
class ConcatenateImages(LeafSystem):
    """Stacks multiple input images into a single output image.

    .. pydrake_system::

        name: ConcatenateImages
        input_ports:
        - color_image_r0_c0
        - ...
        output_ports:
        - color_image
    """

    def __init__(self, *, rows=1, cols=1):
        # TODO(jwnimmer-tri) Add an argument for pixel (image) type.
        LeafSystem.__init__(self)
        assert rows >= 1
        assert cols >= 1
        self._rows = rows
        self._cols = cols
        self._inputs = dict()
        for row in range(rows):
            for col in range(cols):
                key = (row, col)
                self._inputs[key] = self.DeclareAbstractInputPort(
                    name=f"color_image_r{row}_c{col}",
                    model_value=AbstractValue.Make(ImageRgba8U(0, 0)))
        self._output = self.DeclareAbstractOutputPort(
            "color_image",
            alloc=lambda: AbstractValue.Make(ImageRgba8U(0, 0)),
            calc=self._calc_output)

    def get_input_port(self, *, row, col):
        key = (int(row), int(col))
        return self._inputs[key]

    def _calc_output(self, context, output):
        """Implements the output function."""
        result = output.get_mutable_value()
        col_rgbs = []
        for row in range(self._rows):
            row_rgbs = []
            for col in range(self._cols):
                key = (row, col)
                image = self._inputs[key].Eval(context)
                row_rgbs.append(image.data)
            row_rgb = np.hstack(row_rgbs)
            col_rgbs.append(row_rgb)
        rgb = np.vstack(col_rgbs)
        (height, width, _) = rgb.shape
        if result.width() != width or result.height() != height:
            result.resize(width, height)
        result.mutable_data[:] = rgb


class VideoWriter(LeafSystem):
    """Publishes RgbdSensor output to a video file.

    .. pydrake_system::

        name: VideoWriter
        input_ports:
        - color_image

    The video will record images from the color_image input port.

    For companion systems, see also ColorizeDepthImage, ColorizeLabelImage,
    and ConcatenateImages. The function VideoWriter.AddToBuilder makes it
    easy to record color and/or depth and/or labels all at once.

    Note:
        This class uses OpenCV (``import cv2``) to output the video and will
        fail if that module cannot be imported.

    Warning:
        Once all images have been published, you must call
        ``video_writer.Save()`` to finish writing to the file.

    """

    @staticmethod
    def AddToBuilder(*, filename, builder, sensor_pose, fps=16.0,
                     width=320, height=240, fov_y=np.pi/6,
                     near=0.01, far=10.0, kinds=("color",)):
        """Adds a RgbdSensor and VideoWriter system to the given builder.
        Returns the VideoWriter system.

        Args:
            filename: filename to write, e.g., ``"output.mp4"``
            builder: the DiagramBuilder
            sensor_pose: the world-fixed position for the video camera
            fps: the output video's frame rate (in frames per second)
            width: video camera width (in pixels)
            height: video camera width (in pixels)
            fov_y: video camera fov (in radians)
            near: clipping plane distance (in meters)
            far: clipping plane distance (in meters)
            kinds: which image kind(s) to include in the video; valid
                options are color, label, and/or depth
        """
        assert len(kinds) > 0
        assert set(kinds).issubset(set(["color", "label", "depth"]))
        sensor = VideoWriter._AddRgbdSensor(
            builder=builder, pose=sensor_pose,
            width=width, height=height, fov_y=fov_y, near=near, far=far)
        writer = VideoWriter(filename=filename, fps=fps)
        builder.AddSystem(writer)
        if tuple(kinds) == ("color",):
            builder.Connect(sensor.GetOutputPort("color_image"),
                            writer.GetInputPort("color_image"))
            return writer
        stacker = ConcatenateImages(rows=1, cols=len(kinds))
        builder.AddSystem(stacker)
        builder.Connect(stacker.get_output_port(),
                        writer.GetInputPort("color_image"))
        for col, kind in enumerate(kinds):
            stacker_input = stacker.get_input_port(row=0, col=col)
            if kind == "color":
                sensor_output = sensor.GetOutputPort(f"color_image")
                builder.Connect(sensor_output, stacker_input)
            elif kind == "depth":
                sensor_output = sensor.GetOutputPort(f"depth_image_32f")
                converter = ColorizeDepthImage()
                builder.AddSystem(converter)
                builder.Connect(sensor_output, converter.get_input_port())
                builder.Connect(converter.get_output_port(), stacker_input)
            else:
                assert kind == "label"
                sensor_output = sensor.GetOutputPort(f"label_image")
                converter = ColorizeLabelImage()
                builder.AddSystem(converter)
                builder.Connect(sensor_output, converter.get_input_port())
                builder.Connect(converter.get_output_port(), stacker_input)
        return writer

    @staticmethod
    def _AddRgbdSensor(*, builder, pose, width, height, fov_y, near, far):
        """Helper function that adds a fixed-pose RgbdSensor to a scene.
        Returns the sensor system, already added to the builder and connected
        to the scene graph and configured to use the VTK render engine.
        """
        scene_graph = [x for x in builder.GetSystems()
                       if x.get_name() == "scene_graph"][0]
        if not scene_graph.HasRenderer("vtk"):
            scene_graph.AddRenderer("vtk", MakeRenderEngineVtk(
                RenderEngineVtkParams()))
        intrinsics = CameraInfo(width, height, fov_y)
        clip = ClippingRange(near, far)
        camera = DepthRenderCamera(
            RenderCameraCore("vtk", intrinsics, clip, RigidTransform()),
            DepthRange(near, far))
        sensor = RgbdSensor(SceneGraph.world_frame_id(), pose, camera)
        builder.AddSystem(sensor)
        builder.Connect(scene_graph.GetOutputPort("query"),
                        sensor.GetInputPort("geometry_query"))
        return sensor

    def __init__(self, *, filename, fps=16.0):
        """
        In many cases, VideoWriter.AddToBuilder will be easier to use than this
        constructor.

        Args:
            filename: filename to write, e.g., ``"output.mp4"``
            fps: the output video's frame rate (in frames per second)
        """
        LeafSystem.__init__(self)
        self._filename = filename
        self._fps = fps
        self._input = self.DeclareAbstractInputPort(
            name="color_image",
            model_value=AbstractValue.Make(ImageRgba8U(0, 0)))
        self.DeclarePeriodicPublishEvent(1.0 / fps, 0.0, self._publish)
        self._cv2_writer = None

    def Save(self):
        """Flushes all image outputs to the video file and closes the file."""
        if self._cv2_writer is not None:
            self._cv2_writer.release()
        self._cv2_writer = None

    def _publish(self, context):
        color = self._input.Eval(context)
        self._write_frame(rgb=color.data)

    def _write_frame(self, *, rgb):
        # We import cv2 here rather than atop our module so that it will be an
        # optional dependency of pydrake.
        import cv2
        # Open the output file upon the first publish event.
        if self._cv2_writer is None:
            fourcc = cv2.VideoWriter.fourcc("a", "v", "c", "1")
            (height, width, _) = rgb.shape
            self._cv2_writer = cv2.VideoWriter(
                self._filename, fourcc, self._fps, (width, height))
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        self._cv2_writer.write(bgr)
