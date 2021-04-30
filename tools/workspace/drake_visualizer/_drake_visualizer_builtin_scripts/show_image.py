"""
Visualize LCM images from `lcmt_image_array` and `lcmt_image`.

Example usage:

    drake-visualizer \
        --script drake/systems/sensors/visualization/show_images.py

This provides a simple image viewer widget, using portions of code from
director (https://github.com/RobotLocomotion/director, sha: aefc063),
specifically:
*   src/python/director/cameraview.py (CameraImageView)
*   src/app/ddBotImageQueue.cpp
"""
import sys
import argparse
import math
import time
import zlib
import threading

import numpy as np
import numpy.matlib

import vtk
from vtk.util.numpy_support import vtk_to_numpy, get_vtk_array_type

from director import applogic
from director import consoleapp
from director import lcmUtils
from director.timercallback import TimerCallback

import PythonQt
from PythonQt import QtGui

from drake import lcmt_image, lcmt_image_array

from _drake_visualizer_builtin_scripts import scoped_singleton_func

_is_vtk_5 = vtk.vtkVersion().GetVTKMajorVersion() == 5

_verbose = False
_max_depth = -1  # m

DEFAULT_CHANNEL = "DRAKE_RGBD_CAMERA_IMAGES"


class ImageHandler:
    """
    Generic handler to update an image for `ImageWidget`.
    """
    def update_image(self, image):
        """
        This should update `image` in-place, either using `DeepCopy`, or by
        manually resizing, allocating, etc.
        If the image is updated, this should return True.
        Otherwise, it should return False.
        """
        raise NotImplementedError()

    def is_depth_image(self):
        """
        Returns true if the image is a depth image.
        """
        return False


class ImageWidget:
    """
    Wrapper for displaying vtkImageData on a director-style view.

    @note This is more like director's `CameraImageView` than its
    `ImageWidget`.
    """
    def __init__(self, image_handler):
        self._name = 'Image View'
        self._view = PythonQt.dd.ddQVTKWidgetView()
        self._image_handler = image_handler

        self._image = vtk.vtkImageData()
        self._prev_attrib = None

        # Initialize the view.
        self._view.installImageInteractor()
        # Add actor.
        self._image_actor = vtk.vtkImageActor()
        vtk_SetInputData(self._image_actor, self._image)
        self._image_actor.SetVisibility(False)
        self._view.renderer().AddActor(self._image_actor)

        self._view.orientationMarkerWidget().Off()
        self._view.backgroundRenderer().SetBackground(0, 0, 0)
        self._view.backgroundRenderer().SetBackground2(0, 0, 0)

        self._depth_mapper = None

        # Add timer.
        self._render_timer = TimerCallback(
            targetFps=60,
            callback=self.render)
        self._render_timer.start()

    def get_widget(self):
        return self._view

    def render(self):
        if not self._view.isVisible():
            return

        has_new = self._image_handler.update_image(self._image)
        assert isinstance(has_new, bool)
        if not has_new:
            return

        cur_attrib = get_vtk_image_attrib(self._image)
        if self._prev_attrib != cur_attrib:
            if self._prev_attrib is None:
                # Initialization. Ensure it is visible.
                self._image_actor.SetVisibility(True)
            # Fit image to view.
            self._on_new_image_attrib(cur_attrib)
            # Update.
            self._prev_attrib = cur_attrib

        if self._depth_mapper is not None:
            depth_range = self._get_depth_range()
            for i in range(2):
                value = [0.] * 6
                coloring = self._depth_mapper.GetLookupTable()
                coloring.GetNodeValue(i, value)
                value[0] = depth_range[i]
                coloring.SetNodeValue(i, value)

        self._view.render()

    def _get_depth_range(self):
        lower_depth = 0
        upper_depth = _max_depth
        if upper_depth == -1:
            # @note `GetScalarRange` permits non-finite values, such as `inf`.
            # Use a custom mechanism to get min/max.
            data = vtk_image_to_numpy(self._image)
            if data.dtype == np.float32:
                good = np.isfinite(data[:])
            elif data.dtype == np.uint16:
                maxarray = np.full(data.shape, 65535)
                good = np.less(data[:], maxarray)
            else:
                raise RuntimeError(
                    "Unsupported depth format: {}".format(data.dtype))
            if np.any(good):
                upper_depth = np.max(data[good])
        return (lower_depth, upper_depth)

    def _on_new_image_attrib(self, attrib):
        ((w, h, num_channels), dtype) = attrib
        if self._image_handler.is_depth_image():
            assert num_channels == 1, num_channels
            assert dtype in (np.uint16, np.float32), dtype
            # TODO(eric.cousineau): Delegate to outside of `ImageWidget`?
            # This is depth-image specific.
            # For now, just set arbitrary values.

            depth_range = self._get_depth_range()
            lower_color = (1, 1, 1)  # White
            upper_color = (0, 0, 0)  # Black
            nan_color = (0.5, 0.5, 1)  # Light blue - No return.
            inf_color = (0.5, 0, 0.)  # Dark red - Too far / too close.

            # Use `vtkColorTransferFunction` as it provides a more intuitive
            # interpolating interface for me (Eric) than `vtkLookupTable`,
            # since it permits direct specification of RGB values.
            coloring = vtk.vtkColorTransferFunction()
            coloring.AddRGBPoint(depth_range[0], *lower_color)
            coloring.AddRGBPoint(depth_range[1], *upper_color)
            coloring.SetNanColor(*nan_color)
            # @note `coloring.SetAboveRangeColor` doesn't seem to work?
            coloring.AddRGBPoint(depth_range[1] + 10000, *inf_color)
            coloring.SetClamping(True)
            coloring.SetScaleToLinear()

            self._depth_mapper = vtk.vtkImageMapToColors()
            self._depth_mapper.SetLookupTable(coloring)
            vtk_SetInputData(self._depth_mapper, self._image)
            vtk_SetInputData(self._image_actor, self._depth_mapper.GetOutput())
            self._image_actor.GetMapper().SetInputConnection(
                self._depth_mapper.GetOutputPort())
        else:
            # Direct connection.
            self._depth_mapper = None
            vtk_SetInputData(self._image_actor, self._image)

        # Must render first.
        self._view.render()

        # Fit image to view.
        # TODO(eric.cousineau): No idea why this is needed; it worked for
        # VTK 5, but no longer for VTK 6+?
        camera = self._view.camera()
        camera.ParallelProjectionOn()
        camera.SetFocalPoint(0, 0, 0)
        camera.SetPosition(0, 0, -1)
        camera.SetViewUp(0, -1, 0)
        self._view.resetCamera()

        image_height, image_width = get_vtk_image_shape(self._image)[:2]
        view_width, view_height = self._view.renderWindow().GetSize()

        aspect_ratio = float(view_width) / view_height
        parallel_scale = max(image_width / aspect_ratio, image_height) / 2.0
        camera.SetParallelScale(parallel_scale)


class ImageArrayWidget:
    """
    Provides a widget to show images from multiple `ImageHandler`s.
    """
    def __init__(self, handlers):
        # Create widget and layouts
        self._widget = QtGui.QWidget()
        self._image_widgets = list(map(ImageWidget, handlers))
        self._layout = QtGui.QHBoxLayout(self._widget)
        for image_widget in self._image_widgets:
            self._layout.addWidget(image_widget.get_widget())
        self._layout.setContentsMargins(0, 0, 0, 0)

        default_width = 640
        default_height = 480
        dim = [
            default_width * len(self._image_widgets),
            default_height]

        self._widget.resize(*dim)
        self._widget.show()


class DrakeLcmImageViewer:
    """
    Visualize Drake LCM Images.
    """
    def __init__(self, channel=DEFAULT_CHANNEL, frame_names=None):
        """
        @param frame_names
            If None, this will defer creating the subscriber and
            widgets until the first message has been received.
        """
        self._channel = channel
        if frame_names is None:
            self._create_deferred()
        else:
            self._init_full(frame_names)

    def _init_full(self, frame_names):
        self._frame_names = frame_names
        self._subscriber = LcmImageArraySubscriber(
            self._channel, self._frame_names)
        self._widget = ImageArrayWidget(self._subscriber.get_handlers())

    def _create_deferred(self):
        # Defer creating viewer until we have a message.
        def callback(msg):
            # Create.
            frame_names = [image.header.frame_name for image in msg.images]
            print("DrakeLcmImageViewer: Received on '{}', frame_names = {}"
                  .format(self._channel, frame_names))
            self._init_full(frame_names)

        print("DrakeLcmImageViewer: Defer setup until '{}' is received".format(
            self._channel))
        self._defer_sub = lcmUtils.captureMessageCallback(
            self._channel, lcmt_image_array, callback)


def create_image(w, h, num_channels=1, dtype=np.uint8):
    """ Creates a VTK image. """
    image = vtk.vtkImageData()
    image.SetExtent(0, w - 1, 0, h - 1, 0, 0)
    image.SetSpacing(1., 1., 1.)
    image.SetOrigin(0., 0., 0.)
    if _is_vtk_5:
        image.SetWholeExtent(image.GetExtent())
        image.SetScalarType(get_vtk_array_type(dtype))
        image.SetNumberOfScalarComponents(num_channels)
        image.AllocateScalars()
    else:
        image.AllocateScalars(get_vtk_array_type(dtype), num_channels)
    return image


def create_image_if_needed(w, h, num_channels, dtype, image_in):
    """
    Creates a VTK image if `image_in` is not compatible with the desired
    attributes. Otherwise, passes `image_in` through.
    """
    if image_in is not None:
        dim = (w, h, num_channels)
        attrib_out = (dim, dtype)
        attrib_in = get_vtk_image_attrib(image_in)
        if attrib_in == attrib_out:
            return image_in
    # Otherwise, create new image.
    return create_image(w, h, num_channels, dtype)


def vtk_image_to_numpy(image):
    """
    Gets a properly shaped NumPy view of a VTK image's memory with the storage
    format `(h, w, num_channels)`.

    @note This coincides with most other NumPy-based image libraries (OpenCV,
    matplotlib, scipy).
    """
    data = vtk_to_numpy(image.GetPointData().GetScalars())
    data.shape = get_vtk_image_shape(image)
    return data


def get_vtk_image_shape(image):
    """
    Gets `(h, w, num_channels)`.

    @note `vtkImageData.GetDimensions()` returns `(w, h, num_arrays)`, where
    typically `num_arrays == 1 != num_channels`.
    """
    w, h = image.GetDimensions()[:2]
    num_channels = image.GetNumberOfScalarComponents()
    return (h, w, num_channels)


def get_vtk_image_attrib(image):
    """
    Gets `((h, w, num_channels), dtype)` to check if an existing image is
    compatible.
    """
    data = vtk_image_to_numpy(image)
    return (data.shape, data.dtype)


def vtk_SetInputData(obj, input):
    if _is_vtk_5:
        obj.SetInput(input)
    else:
        obj.SetInputData(input)


def decode_lcmt_image(msg, image_in=None):
    """
    Decodes `lcmt_image` to vtkImageData, using an existing image if it is
    compatible.
    """
    enums = lcmt_image
    w = msg.width
    h = msg.height
    pixel_desc = (msg.pixel_format, msg.channel_type)
    if pixel_desc == (enums.PIXEL_FORMAT_RGBA, enums.CHANNEL_TYPE_UINT8):
        num_channels = 4
        dtype = np.uint8
    elif pixel_desc == (enums.PIXEL_FORMAT_DEPTH, enums.CHANNEL_TYPE_FLOAT32):
        num_channels = 1
        dtype = np.float32
    elif pixel_desc == (enums.PIXEL_FORMAT_DEPTH, enums.CHANNEL_TYPE_UINT16):
        num_channels = 1
        dtype = np.uint16
    elif pixel_desc == (enums.PIXEL_FORMAT_LABEL, enums.CHANNEL_TYPE_INT16):
        num_channels = 1
        dtype = np.int16
    else:
        raise RuntimeError("Unsupported pixel type: {}".format(pixel_desc))
    bytes_per_pixel = np.dtype(dtype).itemsize * num_channels
    assert msg.row_stride == msg.width * bytes_per_pixel, msg.row_stride
    if msg.compression_method == enums.COMPRESSION_METHOD_NOT_COMPRESSED:
        data_bytes = msg.data
    elif msg.compression_method == enums.COMPRESSION_METHOD_ZLIB:
        # TODO(eric.cousineau): Consider using `data`s buffer, if possible.
        # Can decompress() somehow use an existing buffer in Python?
        data_bytes = zlib.decompress(msg.data)
    else:
        raise RuntimeError(
            "Unsupported compression type: {}".format(msg.compression_method))
    # Cast to desired type and shape.
    data = np.frombuffer(data_bytes, dtype=dtype)
    data.shape = (h, w, num_channels)
    # Copy data to VTK image.
    image = create_image_if_needed(w, h, num_channels, dtype, image_in)
    image_data = vtk_image_to_numpy(image)
    image_data[:] = data[:]
    return image


class LcmImageHandler(ImageHandler):
    """
    Provides a connection between `LcmImageArraySubscriber`, for a specific
    image frame, and `ImageWidget`.
    """
    def __init__(self):
        self._image = None  # vtkImageData
        self.utime = 0

        self.lock = threading.Lock()
        self.prev_utime = 0
        self._is_depth_image = False

    def receive_message(self, msg):
        """
        Receives and decodes `lcmt_image` message into `vtkImageData`.
        """
        # TODO(eric.cousineau): Consider moving decode logic.
        with self.lock:
            self.utime = msg.header.utime
            self._image = decode_lcmt_image(msg, self._image)
            self._is_depth_image = (msg.pixel_format
                                    == lcmt_image.PIXEL_FORMAT_DEPTH)

    def update_image(self, image_out):
        """
        @see ImageHandler.update_image
        """
        with self.lock:
            if self.utime == self.prev_utime:
                return False
            elif self.utime < self.prev_utime:
                if _verbose:
                    print("Time went backwards. Resetting.")
            self.prev_utime = self.utime
            assert self._image is not None
            image_out.DeepCopy(self._image)
        return True

    def is_depth_image(self):
        return self._is_depth_image


class LcmImageArraySubscriber:
    """
    Provides a connection between the LCM `lcmt_image_array` channel and LCM
    image handlers.
    """
    def __init__(self, channel=DEFAULT_CHANNEL, frame_names=[]):
        self._channel = channel
        self._frame_names = frame_names
        self._handlers = {}
        for frame_name in self._frame_names:
            self._handlers[frame_name] = LcmImageHandler()
        self._subscriber = lcmUtils.addSubscriber(
            channel, lcmt_image_array, self._on_message)

    def get_handlers(self):
        """ Returns ordered list of handlers. """
        # TODO(eric.cousineau): Consider just using `OrderedDict`.
        return map(self._handlers.get, self._frame_names)

    def _on_message(self, msg):
        issues = []

        msg_frame_names = [image.header.frame_name for image in msg.images]
        for (frame_name, handler) in self._handlers.items():
            if frame_name not in msg_frame_names:
                issues.append("Did not find '{}' in message"
                              .format(frame_name))
                continue
            index = msg_frame_names.index(frame_name)
            msg_image = msg.images[index]
            handler.receive_message(msg_image)

        if _verbose:
            extra_frame_names = set(msg_frame_names) - set(self._frame_names)
            for frame_name in extra_frame_names:
                issues.append("Got extra image '{}'".format(frame_name))

        if issues:
            print("LcmImageArraySubscriber: For image channel '{}' "
                  "({} images), with frames {}:"
                  .format(self._channel, msg.num_images, msg_frame_names))
            for issue in issues:
                print("  {}".format(issue))


class TestImageHandler(ImageHandler):
    """
    Provides a simple test image handler.
    This just shows an animated gradient, either in color or in grayscale.
    """
    def __init__(self, do_color):
        self.do_color = do_color
        self.start_time = time.time()
        if self.do_color:
            self._image = create_image(640, 480, 4, dtype=np.uint8)
        else:
            self._image = create_image(640, 480, 1, dtype=np.float32)
        self.has_switched = False

    def update_image(self, image_out):
        t = time.time() - self.start_time

        if t > 2 and not self.has_switched:
            # Try changing the size.
            if self.do_color:
                self._image = create_image(480, 640, 4, dtype=np.uint8)
            else:
                self._image = create_image(480, 640, 1, dtype=np.float32)
            self.has_switched = True

        if self.do_color:
            p = 255.
        else:
            p = 10.  # m

        data = vtk_image_to_numpy(self._image)
        h, w = data.shape[:2]

        x = np.matlib.repmat(
            np.linspace(0, 1, w).reshape(1, -1), h, 1)
        y = np.matlib.repmat(
            np.linspace(0, 1, h).reshape(-1, 1), 1, w)
        T = 1
        w = 2 * math.pi / T
        s = (math.sin(w * t) + 1) / 2.
        s2 = (math.sin(w * t * 5) + 1) / 2.

        data[:, :, 0] = p * y * s
        if self.do_color:
            data[:, :, 1] = p * (1 - y)
            data[:, :, 2] = p * x * s2
            data[:, :, 3] = p * s
        else:
            # Test `inf` and `nan`.
            gw = 50
            data[0:gw, 0:gw, 0] = np.inf
            data[0:gw, gw:(2 * gw), 0] = np.nan

        image_out.DeepCopy(self._image)
        return True


@scoped_singleton_func
def init_visualizer(debug=False):
    if not debug:
        return DrakeLcmImageViewer(DEFAULT_CHANNEL)
    else:
        print("Using test image viewer")
        return ImageArrayWidget([
            TestImageHandler(do_color=True),
            TestImageHandler(do_color=False),
            ])


# Activate the plugin if this script is run directly; store the results to keep
# the plugin objects in scope.
if __name__ == "__main__":
    image_viz = init_visualizer()
