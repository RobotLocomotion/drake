# Note that this script runs in the main context of drake-visulizer,
# where many modules and variables already exist in the global scope.
from sets import Set
from director import lcmUtils
from director import applogic
from director import objectmodel as om
from director import visualization as vis
import robotlocomotion as lcmrobotlocomotion


class FrameChannel(object):

    def __init__(self, parent_folder, channel):
        self._parent_folder = parent_folder
        self._channel = channel
        # Link names that were previously published.
        self._link_name_published = []

    def handle_message(self, msg):
        if set(self._link_name_published) != set(msg.link_name):
            # Removes the folder completely.
            self.remove_folder()
            self._link_name_published = msg.link_name

        folder = self._get_folder()

        for i in range(0, msg.num_links):
            name = msg.link_name[i]
            transform = transformUtils.transformFromPose(
                msg.position[i], msg.quaternion[i])
            # `vis.updateFrame` will either create or update the frame
            # according to its name within its parent folder.
            vis.updateFrame(transform, name, parent=folder, scale=0.1)

    def _get_folder(self):
        return om.getOrCreateContainer(
            self._channel, parentObj=self._parent_folder)

    def remove_folder(self):
        om.removeFromObjectModel(self._get_folder())


class FramesVisualizer(object):

    def __init__(self):
        self._name = "Frame Visualizer"
        self._subscriber = None
        self._frame_channels = {}
        self.set_enabled(True)

    def _add_subscriber(self):
        if (self._subscriber is not None):
            return

        self._subscriber = lcmUtils.addSubscriber(
            'DRAKE_DRAW_FRAMES.*',
            messageClass=lcmrobotlocomotion.viewer_draw_t,
            callback=self._handle_message,
            callbackNeedsChannel=True)
        self._subscriber.setNotifyAllMessagesEnabled(True)

    def _get_folder(self):
        return om.getOrCreateContainer(self._name)

    def _remove_subscriber(self):
        if (self._subscriber is None):
            return
        lcmUtils.removeSubscriber(self._subscriber)
        for frame_channel in self._frame_channels:
            frame_channel.remove_folder()
        self._frame_channels.clear()
        self._subscriber = None
        om.removeFromObjectModel(self._get_folder())

    def is_enabled(self):
        return self._subscriber is not None

    def set_enabled(self, enable):
        if enable:
            self._add_subscriber()
        else:
            self._remove_subscriber()

    def _handle_message(self, msg, channel):
        frame_channel = self._frame_channels.get(channel)
        if not frame_channel:
            frame_channel = FrameChannel(
                parent_folder=self._get_folder(), channel=channel)
            self._frame_channels[channel] = frame_channel
        frame_channel.handle_message(msg)


def init_visualizer():
    frame_viz = FramesVisualizer()

    # Adds to the "Tools" menu.
    applogic.MenuActionToggleHelper(
        'Tools', frame_viz._name,
        frame_viz.is_enabled, frame_viz.set_enabled)
    return frame_viz


# Creates the visualizer when this script is executed.
frame_viz = init_visualizer()
