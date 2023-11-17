import copy
import numpy as np

from pydrake.common.value import Value
from pydrake.geometry import (
    ClippingRange,
    DepthRange,
    DepthRenderCamera,
    MakeRenderEngineVtk,
    RenderCameraCore,
    RenderEngineVtkParams,
    SceneGraph,
)
from pydrake.math import RigidTransform
from pydrake.systems.framework import LeafSystem
from pydrake.systems.sensors import (
    CameraInfo,
    ImageRgba8U,
    RgbdSensor,
)
from pydrake.visualization import (
    ColorizeDepthImage,
    ColorizeLabelImage,
    ConcatenateImages,
)


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

    def __init__(self, *, filename, fps=16.0, backend="PIL", fourcc=None):
        """Constructs a VideoWriter system.

        In many cases, the AddToBuilder() or ConnectRgbdSensor() methods might
        be easier to use than this constructor.

        Args:
            filename: filename to write, e.g., ``"output.gif"`` or
                ``"output.mp4"``
            fps: the output video's frame rate (in frames per second)
            backend: which backend to use: "PIL" or "cv2"
            fourcc: when using the cv2 backend, which encoder to use;
                good choices are "mp4v" or "avc1"; defaults to "mp4v";
                refer to the OpenCV documentation for details.
        """
        LeafSystem.__init__(self)
        self._filename = filename
        self._fps = fps
        self._input = self.DeclareAbstractInputPort(
            name="color_image",
            model_value=Value(ImageRgba8U()))
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
            self._fourcc = fourcc or "mp4v"
            if len(self._fourcc) != 4:
                raise ValueError(f"The fourcc={fourcc!r} must be 4 characters")
        else:
            raise RuntimeError(f"Invalid backend={backend!r}")

    @staticmethod
    def AddToBuilder(*, filename, builder, sensor_pose, fps=16.0,
                     width=320, height=240, fov_y=np.pi/6, near=0.01, far=10.0,
                     kinds=None, backend="PIL", fourcc=None):
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
            fourcc: when using the cv2 backend, which encoder to use;
                good choices are "mp4v" or "avc1"; defaults to "mp4v"
                refer to the OpenCV documentation for details.

        Warning:
            Once all images have been published, you must call
            ``video_writer.Save()`` to finish writing to the video file.
        """
        sensor = VideoWriter._AddRgbdSensor(
            builder=builder, pose=sensor_pose,
            width=width, height=height, fov_y=fov_y, near=near, far=far)
        writer = VideoWriter(filename=filename, fps=fps, backend=backend,
                             fourcc=fourcc)
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
                    sensor.GetOutputPort("depth_image_32f"),
                    converter.GetInputPort("depth_image_32f"))
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
            fourcc = cv2.VideoWriter.fourcc(*self._fourcc)
            (height, width, _) = rgba.shape
            self._cv2_writer = cv2.VideoWriter(
                self._filename, fourcc, self._fps, (width, height))
        bgra = cv2.cvtColor(rgba, cv2.COLOR_RGB2BGR)
        self._cv2_writer.write(bgra)
