#!/usr/bin/env python

"""
This is a simple script to convert from robotlocomotion (rl) LCM image types to
`bot_core` (bc), so that we may leverage existing director code for
visualization.

To run this:

    drake-visualizer --script ./show_images.py --bot-config ./bot_frames.cfg

Note that if you use a non-default --channel, then you should use the
appropriate --bot-config file (stored via `director.drcargs`). For more
information about this file, look at the comments in `bot_frames.cfg`.
(In the future, the better solution would be to remove dependence on
bot-frames).

@see director.imageviewerapp.ImageViewApp
"""

import zlib
import numpy as np
import argparse

import PythonQt
from PythonQt import QtGui

from director import consoleapp
from director import cameraview
from director import applogic
from director import vtkAll as vtk
from director import lcmUtils

import bot_core as bc
import robotlocomotion as rl

# TODO(eric.cousineau): This is almost directly derived from
# directory.imageviewapp.ImageViewApp.
# This could be avoided by permitting other options to be passed to
# ImageViewApp, such that the translator could be handled separately.


class DrakeImageViewer(object):
    def __init__(self):
        self.setup()

    def addShortcuts(self):
        applogic.addShortcut(self.widget, 'Ctrl+Q', consoleapp.ConsoleApp.quit)
        applogic.addShortcut(self.widget, 'F8',
                             consoleapp.ConsoleApp.showPythonConsole)

    def parseArgs(self, defaultChannel='DRAKE_RGBD_CAMERA_IMAGES'):
        parser = argparse.ArgumentParser()
        parser.add_argument('--channel', type=str, help='image channel',
                            default=defaultChannel)
        args, unknown = parser.parse_known_args()
        return args

    def setup(self):
        args = self.parseArgs()
        self.imageManager = cameraview.ImageManager()

        # Redirect bot_core duplicates to a separate channel.
        channel = args.channel + "_BOT_CORE"
        # Add LCM translator.
        self.translator = addTranslator(args.channel, channel)

        self.views = []

        # Add stream for color image.
        imageName = channel + '_RGB'
        self.imageManager.queue.addCameraStream(channel, imageName, 0)
        self.imageManager.addImage(imageName)
        self.cameraView = cameraview.CameraImageView(
            self.imageManager, imageName,
            view=PythonQt.dd.ddQVTKWidgetView())
        self.cameraView.eventFilterEnabled = False
        v = self.cameraView.view
        v.renderWindow().GetInteractor().SetInteractorStyle(
            vtk.vtkInteractorStyleImage())
        self.views.append(self.cameraView.view)

        # Add stream for depth image.
        imageName2 = channel + '_D'
        self.imageManager.queue.addCameraStream(
            channel, imageName2, bc.images_t.DEPTH_MM_ZIPPED)
        self.imageManager.addImage(imageName2)
        self.cameraView2 = cameraview.CameraImageView(
            self.imageManager, imageName2,
            view=PythonQt.dd.ddQVTKWidgetView())
        self.cameraView2.eventFilterEnabled = False
        self.cameraView2.useImageColorMap = True
        v = self.cameraView2.view
        v.renderWindow().GetInteractor().SetInteractorStyle(
            vtk.vtkInteractorStyleImage())
        self.views.append(self.cameraView2.view)

        self.widget = QtGui.QWidget()
        self.layout = QtGui.QHBoxLayout(self.widget)
        for view in self.views:
            self.layout.addWidget(view)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.widget.resize(640*len(self.views), 480)
        self.widget.show()
        self.addShortcuts()


def addTranslator(rl_channel, bc_channel):
    """
    Creates a subscriber that translates from robotlocomotion (rl) to bot_core
    (bc) image types.

    Presently, ddBotImageQueue (in director) uses the specific image formats
    handled by `rl_to_bc_image_array_t` and `rl_to_bc_image_t`.
    """
    def callback(rl_msg):
        bc_msg = rl_to_bc_image_array_t(rl_msg)
        lcmUtils.publish(bc_channel, bc_msg)
    return lcmUtils.addSubscriber(rl_channel, rl.image_array_t, callback)


def rl_to_bc_image_array_t(rl_msg):
    """ Converts image array. """
    assert(isinstance(rl_msg, rl.image_array_t))
    bc_msg = bc.images_t()
    bc_msg.utime = rl_msg.header.utime
    for rl_img in rl_msg.images:
        info = rl_to_bc_image_t(rl_img)
        if info is None:
            continue
        (bc_img, bc_img_type) = info
        bc_msg.images.append(bc_img)
        bc_msg.image_types.append(bc_img_type)
    bc_msg.n_images = len(bc_msg.images)
    return bc_msg


def rl_to_bc_image_t(rl_msg):
    """ Converts single image. """
    # Convenience aliases for enumerations.
    rli = rl.image_t
    bci = bc.image_t
    bcis = bc.images_t

    assert(isinstance(rl_msg, rl.image_t))
    bc_msg = bc.image_t()
    bc_msg.utime = rl_msg.header.utime
    bc_msg.width = rl_msg.width
    bc_msg.height = rl_msg.height
    assert(rl_msg.compression_method == rli.COMPRESSION_METHOD_ZLIB)
    pixel_desc = (rl_msg.pixel_format, rl_msg.channel_type)
    if pixel_desc == (rli.PIXEL_FORMAT_RGBA, rli.CHANNEL_TYPE_UINT8):
        # Color image.
        bc_msg.pixelformat = bci.PIXEL_FORMAT_RGB
        bc_img_type = bcis.LEFT
        rgba_to_rgb(rl_msg, bc_msg)
    elif pixel_desc == (rli.PIXEL_FORMAT_DEPTH, rli.CHANNEL_TYPE_FLOAT32):
        # Depth image.
        bc_msg.pixelformat = bci.PIXEL_FORMAT_GRAY
        bc_img_type = bcis.DEPTH_MM_ZIPPED
        depth32f_to_depthmm(rl_msg, bc_msg)
    elif pixel_desc == (rli.PIXEL_FORMAT_LABEL, rli.CHANNEL_TYPE_INT16):
        # Label image (ignored).
        # TODO(eric.cousineau): Consider palettizing this here or elsewhere?
        return None
    else:
        raise Exception("Unsupported pixel type: {}".format(pixel_desc))
    bc_msg.size = len(bc_msg.data)
    return (bc_msg, bc_img_type)


def rgba_to_rgb(rl_msg, bc_msg):
    """ Converts uint8 RGBA to uint8 RGB. """
    assert(rl_msg.row_stride == 4 * rl_msg.width)
    num_pixels = rl_msg.width * rl_msg.height
    raw_in = np.frombuffer(zlib.decompress(rl_msg.data), dtype=np.uint8)
    rgba_in = np.reshape(raw_in, (num_pixels, 4))
    rgb_out = rgba_in[:, 0:3]
    bc_msg.data = rgb_out.tobytes()
    bc_msg.row_stride = 3 * rl_msg.width


def depth32f_to_depthmm(rl_msg, bc_msg):
    """ Converts Depth32F to bot_core depth image type. """
    assert(rl_msg.row_stride == 4 * rl_msg.width)
    num_pixels = rl_msg.width * rl_msg.height
    raw_in = np.frombuffer(zlib.decompress(rl_msg.data), dtype=np.float32)
    depth32f_in = np.reshape(raw_in, num_pixels)
    depthmm_out = np.zeros(num_pixels, dtype=np.uint16)
    depthmm_out[:] = 1000 * depth32f_in
    bc_msg.data = zlib.compress(depthmm_out.tobytes())
    bc_msg.row_stride = 2 * rl_msg.width


def main():
    viewer = DrakeImageViewer()
    globals().update(image_viewer=viewer)

    # Start app if needed (e.g., if using `directorPython`).
    has_app = 'app' in globals()
    if not has_app:
        app = consoleapp.ConsoleApp()
        app.start()


if __name__ == '__main__':
    main()
