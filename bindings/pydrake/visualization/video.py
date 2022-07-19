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


class VideoWriter(LeafSystem):
    """Publishes RgbdSensor output to a video file.

    .. pydrake_system::

        name: VideoWriter
        input_ports:
        - color_image
        - depth_image_32f
        - label_image

    The video will include images from any of the three input ports that are
    connected. In most cases, only the color_image port will be connected.

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
        sensor = VideoWriter._AddRgbdSensor(
            builder=builder, pose=sensor_pose,
            width=width, height=height, fov_y=fov_y, near=near, far=far)
        writer = VideoWriter(filename=filename, fps=fps)
        builder.AddSystem(writer)
        for kind in kinds:
            if kind == "depth":
                port_name = "depth_image_32f"
            else:
                port_name = f"{kind}_image"
            builder.Connect(sensor.GetOutputPort(port_name),
                            writer.GetInputPort(port_name))
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
        self._color_input = self.DeclareAbstractInputPort(
            name="color_image",
            model_value=AbstractValue.Make(ImageRgba8U(0, 0)))
        self._depth_input = self.DeclareAbstractInputPort(
            name="depth_image_32f",
            model_value=AbstractValue.Make(ImageDepth32F(0, 0)))
        self._label_input = self.DeclareAbstractInputPort(
            name="label_image",
            model_value=AbstractValue.Make(ImageLabel16I(0, 0)))
        self.DeclarePeriodicPublishEvent(1.0 / fps, 0.0, self._publish)
        self._cv2_writer = None

    def Save(self):
        """Flushes all image outputs to the video file and closes the file."""
        if self._cv2_writer is not None:
            self._cv2_writer.release()
        self._cv2_writer = None

    def _publish(self, context):
        color = depth = label = None
        if self._color_input.HasValue(context):
            color = self._color_input.Eval(context)
        if self._depth_input.HasValue(context):
            depth = self._depth_input.Eval(context)
        if self._label_input.HasValue(context):
            label = self._label_input.Eval(context)
        num_kinds = sum([x is not None for x in [color, depth, label]])
        assert num_kinds > 0
        some_image = color or depth or label
        width, height = some_image.width(), some_image.height()
        self._write_frame(num_kinds, width, height, color, depth, label)
        return EventStatus.Succeeded()

    def _write_frame(self, num_kinds, width, height, color, depth, label):
        # We import cv2 here rather than atop our module so that it will be an
        # optional dependency of pydrake.
        import cv2
        # Open the output file upon the first publish event.
        if self._cv2_writer is None:
            fourcc = cv2.VideoWriter.fourcc("a", "v", "c", "1")
            self._cv2_writer = cv2.VideoWriter(
                self._filename, fourcc, self._fps,
                (width * num_kinds, height))
        # Add the color image.
        bgrs = []
        if color:
            rgb = color.data
            bgrs.append(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        if depth:
            rgb = self._colorize_depth(depth.data.squeeze(2))
            bgrs.append(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        if label:
            rgb = self._colorize_label(label.data.squeeze(2))
            bgrs.append(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        self._cv2_writer.write(np.hstack(bgrs))

    @staticmethod
    def _colorize_depth(depth, invalid_color=(100, 0, 0)):
        assert depth.dtype == np.float32
        invalid = (depth <= 0) | ~np.isfinite(depth)
        scale_min = np.min(depth[~invalid])
        scale_max = np.max(depth[~invalid])
        # Normalize.
        depth_scaled = (depth - scale_min) / (scale_max - scale_min)
        h, w = depth_scaled.shape
        # Convert to rgb.
        depth_rgb = np.tile(depth_scaled.reshape((h, w, 1)), (1, 1, 3))
        # Invert coloring (white is up front).
        depth_rgb = 1 - depth_rgb
        # Recolor.
        depth_rgb = (depth_rgb * 255).astype(np.uint8)
        depth_rgb[invalid] = np.array(invalid_color)
        return depth_rgb

    @staticmethod
    def _palette():
        values = []
        values.extend(mpl.colors.TABLEAU_COLORS.values())
        values.extend(mpl.colors.BASE_COLORS.values())
        cc = mpl.colors.ColorConverter()
        rgb_float = np.array([cc.to_rgb(x) for x in values])
        return (rgb_float*255).astype(np.uint8)

    @staticmethod
    def _colorize_label(label):
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
        palette = VideoWriter._palette()
        color_image = palette[label.squeeze() % len(palette)]
        color_image[background] = [0, 0, 0]
        return color_image
