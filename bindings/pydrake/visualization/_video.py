import copy
import matplotlib as mpl
import numpy as np

from pydrake.common.value import AbstractValue
from pydrake.geometry import (
    Rgba,
    SceneGraph,
)
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
from pydrake.systems.framework import LeafSystem
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

    Depth measurements are linearly mapped to a grayscale palette, with smaller
    (closer) values brighter and larger (further) values darker.

    The dynamic range of each input image determines the scale. The pixel with
    the smallest depth will be fully white (#FFFFFFFF), and largest depth will
    be fully black (#000000FF). Note that alpha channel is still 100% in both
    cases.

    Because the dynamic range is measured one input image a time, take note
    that a video recording of this System will not have consistent scaling
    across its entirety. The ability to set a fixed palette is future work.

    For the special depth pixel values "too close" or "too far", the color
    pixel will use the ``invalid_color`` property (by default, a dim red).
    """

    def __init__(self):
        LeafSystem.__init__(self)
        self._depth32_input = self.DeclareAbstractInputPort(
            name="depth_image_32f",
            model_value=AbstractValue.Make(ImageDepth32F()))
        self._color_output = self.DeclareAbstractOutputPort(
            "color_image",
            alloc=lambda: AbstractValue.Make(ImageRgba8U()),
            calc=self._calc_output)
        self.invalid_color = Rgba(100/255, 0, 0, 1)

    def _calc_output(self, context, output):
        """Implements the color_image output calculation."""
        depth = self._depth32_input.Eval(context)
        color = output.get_mutable_value()
        self._colorize_depth_image(depth, color)

    def _colorize_depth_image(self, depth, color):
        """Colorizes an ImageDepth32F into an ImageRgba8U.
        The color is an output argument; there is no return value.
        """
        if not all([color.width() == depth.width(),
                    color.height() == depth.height()]):
            color.resize(depth.width(), depth.height())
        depth_array = depth.data.squeeze(2)
        color.mutable_data[:] = self._colorize_depth_array(depth_array)

    def _colorize_depth_array(self, depth):
        """Colorizes an np.array of depths into an np.array of rgba.
        Returns the color array.
        """
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
        color[invalid] = self._invalid_color_pixel()
        return color

    def _invalid_color_pixel(self):
        """Converts the Rgba self.invalid_color to an np.uint8 array."""
        return np.array([
            self.invalid_color.r() * 255,
            self.invalid_color.g() * 255,
            self.invalid_color.b() * 255,
            self.invalid_color.a() * 255,
        ], dtype=np.uint8)


# TODO(jwnimmer-tri) Move this system to C++ so everyone can use it.
class ColorizeLabelImage(LeafSystem):
    """Converts a label image to a color image.

    .. pydrake_system::

        name: ColorizeLabelImage
        input_ports:
        - label_image
        output_ports:
        - color_image

    Labels are mapped to colors with a built-in, fixed palette.

    For label pixels that do not represent a label ("don't care", "empty",
    "unspecified", etc.), the color pixel will use the ``background_color``
    property (by default, black with 0% alpha).
    """

    def __init__(self):
        LeafSystem.__init__(self)
        self._label_input = self.DeclareAbstractInputPort(
            name="label_image",
            model_value=AbstractValue.Make(ImageLabel16I()))
        self._color_output = self.DeclareAbstractOutputPort(
            "color_image",
            alloc=lambda: AbstractValue.Make(ImageRgba8U()),
            calc=self._calc_output)
        self._palette = self._make_palette()
        self.background_color = Rgba(0, 0, 0, 0)

    def _calc_output(self, context, output):
        """Implements the color_image output calculation."""
        label = self._label_input.Eval(context)
        color = output.get_mutable_value()
        self._colorize_label_image(label, color)

    def _colorize_label_image(self, label, color):
        """Colorizes an ImageLabel16I into an ImageRgba8U.
        The color is an output argument; there is no return value.
        """
        if not all([color.width() == label.width(),
                    color.height() == label.height()]):
            color.resize(label.width(), label.height())
        label_array = label.data.squeeze(2)
        color.mutable_data[:] = self._colorize_label_array(label_array)

    def _colorize_label_array(self, label):
        """Colorizes an np.array of labels into an np.array of rgba.
        Returns the color array.
        """
        assert label.dtype == np.int16
        background = np.zeros(label.shape[:2], dtype=bool)
        reserved_labels = [
            RenderLabel.kDoNotRender,
            RenderLabel.kDontCare,
            RenderLabel.kEmpty,
            RenderLabel.kUnspecified,
        ]
        for x in reserved_labels:
            reserved_mask = label == int(x)
            background |= reserved_mask
        color = self._palette[label.squeeze() % len(self._palette)]
        color[background] = self._background_color_pixel()
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

    def _background_color_pixel(self):
        """Converts the Rgba self.background_color to an np.uint8 array."""
        return np.array([
            self.background_color.r() * 255,
            self.background_color.g() * 255,
            self.background_color.b() * 255,
            self.background_color.a() * 255,
        ], dtype=np.uint8)


# TODO(jwnimmer-tri) Move this system to C++ so everyone can use it.
class ConcatenateImages(LeafSystem):
    """Stacks multiple input images into a single output image.

    .. pydrake_system::

        name: ConcatenateImages
        input_ports:
        - color_image_r0_c0
        - color_image_r0_c1
        - ...
        output_ports:
        - color_image

    As currently implemented, all inputs must be of type ImageRgba8U, must
    have identical width and height, and all ports must be connected (i.e.,
    with no gaps in the grid of rows x cols). Any of those conditions could
    be generalized as future work.
    """

    def __init__(self, *, rows=1, cols=1):
        """Constructs a ConcatenateImages system.

        Args:
            rows: Number of images to stack vertically.
            cols: Number of images to stack horizontally.
        """
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
                    model_value=AbstractValue.Make(ImageRgba8U()))
        self._output = self.DeclareAbstractOutputPort(
            "color_image",
            alloc=lambda: AbstractValue.Make(ImageRgba8U()),
            calc=self._calc_output)

    def get_input_port(self, *, row, col):
        """Returns the InputPort for the given (row, col) image.

        Rows and columns are 0-indexed, i.e., we have ``0 <= row < rows`` and
        ``0 <= col < cols``.
        """
        key = (int(row), int(col))
        return self._inputs[key]

    def _calc_output(self, context, output):
        """Implements the output calculation."""
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

    The video will contain recorded images from the color_image input port.

    The methods AddToBuilder() or ConnectToRgbdSensor() make it easy to record
    color and/or depth and/or labels all at once.  For companion systems, see
    also ColorizeDepthImage, ColorizeLabelImage, and ConcatenateImages.

    This class delegates actual video output to one of two video backends,
    either "PIL" (aka Pillow) or "cv2". PIL generally only supports image
    formats (e.g., gif, apng, webp) while cv2 also supports movies (e.g., mp4).

    Warning:
        Once all images have been published, you must call
        ``video_writer.Save()`` to finish writing to the video file.

    Warning:
        This class will fail at construction time if the specified ``backend``
        module cannot be imported. You must ensure that whichever backend you
        choose is available in your environment. Drake neither bundles nor
        depends on either one.
    """

    def __init__(self, *, filename, fps=16.0, backend="PIL"):
        """Constructs a VideoWriter system.

        In many cases, the AddToBuilder() or ConnectRgbdSensor() methods might
        be easier to use than this constructor.

        Args:
            filename: filename to write, e.g., ``"output.gif"`` or
                ``"output.mp4"``
            fps: the output video's frame rate (in frames per second)
            backend: which backend to use: "PIL" or "cv2"
        """
        LeafSystem.__init__(self)
        self._filename = filename
        self._fps = fps
        self._input = self.DeclareAbstractInputPort(
            name="color_image",
            model_value=AbstractValue.Make(ImageRgba8U()))
        # TODO(jwnimmer-tri) Support forced triggers as well (so users can
        # manually record videos of prescribed motion).
        self.DeclarePeriodicPublishEvent(1.0 / fps, 0.0, self._publish)
        self._cv2_writer = None
        self._pil_images = None
        if backend == "PIL":
            from PIL import Image
            self._backend = Image
            self._write = self._write_pil
        elif backend == "cv2":
            import cv2
            self._backend = cv2
            self._write = self._write_cv2
        else:
            raise RuntimeError(f"Invalid backend={backend!r}")

    @staticmethod
    def AddToBuilder(*, filename, builder, sensor_pose, fps=16.0,
                     width=320, height=240, fov_y=np.pi/6,
                     near=0.01, far=10.0, kinds=None, backend="PIL"):
        """Adds a RgbdSensor and VideoWriter system to the given builder, using
        a world-fixed pose. Returns the VideoWriter system.

        See also ConnectRgbdSensor() in case you want to attach a VideoWriter
        to an already existing RgbdSensor (e.g., on attached to a robot).

        Args:
            filename: filename to write, e.g., ``"output.gif"`` or
                ``"output.mp4"``
            builder: the DiagramBuilder
            sensor_pose: the world-fixed position for the video camera
            fps: the output video's frame rate (in frames per second)
            width: video camera width (in pixels)
            height: video camera width (in pixels)
            fov_y: video camera fov (in radians)
            near: clipping plane distance (in meters)
            far: clipping plane distance (in meters)
            kinds: which image kind(s) to include in the video; valid
                options are ``"color"``, ``"label"``, and/or ``"depth"``
            backend: which backend to use: "PIL" or "cv2".

        Warning:
            Once all images have been published, you must call
            ``video_writer.Save()`` to finish writing to the video file.
        """
        sensor = VideoWriter._AddRgbdSensor(
            builder=builder, pose=sensor_pose,
            width=width, height=height, fov_y=fov_y, near=near, far=far)
        writer = VideoWriter(filename=filename, fps=fps, backend=backend)
        builder.AddSystem(writer)
        writer.ConnectRgbdSensor(builder=builder, sensor=sensor, kinds=kinds)
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

    def ConnectRgbdSensor(self, *, builder, sensor, kinds=None):
        """Adds a VideoWriter system to the given builder and connects it to
        the given RgbdSensor. Returns the VideoWriter system.

        See also AddToBuilder() in case you want to record video from a world-
        fixed pose by creating a new sensor.

        Args:
            builder: the DiagramBuilder
            kinds: which image kind(s) to include in the video; valid
                options are ``"color"``, ``"label"``, and/or ``"depth"``;
                when set to ``None``, defaults to ``"color"`` only.

        Warning:
            Once all images have been published, you must call
            ``video_writer.Save()`` to finish writing to the video file.
        """
        # Make a list of ImageRgba8U output ports to feed as video input.
        image_sources = []
        for kind in (kinds or ("color",)):
            if kind == "color":
                image_sources.append(sensor.GetOutputPort("color_image"))
            elif kind == "depth":
                converter = builder.AddSystem(ColorizeDepthImage())
                builder.Connect(
                    sensor.GetOutputPort(f"depth_image_32f"),
                    converter.get_input_port())
                image_sources.append(converter.get_output_port())
            elif kind == "label":
                converter = builder.AddSystem(ColorizeLabelImage())
                builder.Connect(
                    sensor.GetOutputPort(f"label_image"),
                    converter.get_input_port())
                image_sources.append(converter.get_output_port())
            else:
                raise RuntimeError(f"Unknown image kind={kind!r}")
        num_sources = len(image_sources)
        if num_sources == 1:
            image_source = image_sources[0]
        else:
            stacker = builder.AddSystem(ConcatenateImages(cols=num_sources))
            for i, source in enumerate(image_sources):
                builder.Connect(source, stacker.get_input_port(row=0, col=i))
            image_source = stacker.get_output_port()
        builder.Connect(image_source, self.get_input_port())

    def Save(self):
        """Flushes all images to the video file and closes the file.

        Warning:
            Continuing a simulation after calling Save() will begin to
            overwrite the prior video with a new one.
        """
        # For PIL.
        if self._pil_images is not None:
            images = self._pil_images
            frame_millis = int(1000.0 / self._fps)
            images[0].save(
                self._filename, save_all=True, append_images=images[1:],
                optimize=True, duration=frame_millis)
        self._pil_images = None
        # For cv2.
        if self._cv2_writer is not None:
            self._cv2_writer.release()
        self._cv2_writer = None

    def _publish(self, context):
        """The framework event handler that saves one input image."""
        color = self._input.Eval(context)
        # Call the backend-specific function that was set by our constructor.
        self._write(rgba=color.data)

    def _write_pil(self, *, rgba):
        """Saves one input image (when we're configured to use PIL)."""
        # Grab the `from PIL import Image` that we stored at construction-time.
        Image = self._backend
        image = Image.fromarray(copy.copy(rgba), mode="RGBA")
        if self._pil_images is None:
            self._pil_images = [image]
        else:
            self._pil_images.append(image)

    def _write_cv2(self, *, rgba):
        """Saves one input image (when we're configured to use cv2)."""
        # Grab `import cv2` that we stored at construction-time.
        cv2 = self._backend
        # Open the output file upon the first publish event.
        if self._cv2_writer is None:
            fourcc = cv2.VideoWriter.fourcc("a", "v", "c", "1")
            (height, width, _) = rgba.shape
            self._cv2_writer = cv2.VideoWriter(
                self._filename, fourcc, self._fps, (width, height))
        bgra = cv2.cvtColor(rgba, cv2.COLOR_RGB2BGR)
        self._cv2_writer.write(bgra)
