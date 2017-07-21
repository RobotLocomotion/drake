# Note that this script runs in the main context of drake-visulizer,
# where many modules and variables already exist in the global scope.
from sets import Set
from director import lcmUtils
from director import applogic
from director import objectmodel as om
from director import visualization as vis
import robotlocomotion as lcmrobotlocomotion

class FramesVisualizer(object):
    def __init__(self):
        self._folder_name = 'Frames'
        self._name = "Frame Visualizer"
        self._subscriber = None
        self.set_enabled(True)

    def add_subscriber(self):
        if (self._subscriber is not None):
            return

        self._subscriber = lcmUtils.addSubscriber(
            'DRAKE_DRAW_FRAMES',
            messageClass = lcmrobotlocomotion.viewer_draw_t,
            callback = self.handle_message)

    def remove_subscriber(self):
        if (self._subscriber is None):
            return

        lcmUtils.removeSubscriber(self._subscriber)
        om.removeFromObjectModel(om.findObjectByName(self._folder_name))
        self._subscriber = None

    def is_enabled(self):
        return self._subscriber is not None

    def set_enabled(self, enable):
        if enable:
            self.add_subscriber()
        else:
            self.remove_subscriber()

    def handle_message(self, msg):
        # Removes the folder completely.
        om.removeFromObjectModel(om.findObjectByName(self._folder_name))

        # Recreates folder.
        folder = om.getOrCreateContainer(self._folder_name)

        for i in range(0, msg.num_links):
            name = msg.link_name[i]
            pos = [msg.position[i][0], msg.position[i][1], msg.position[i][2]]
            quat = [msg.quaternion[i][0], msg.quaternion[i][1], msg.quaternion[i][2], msg.quaternion[i][3]]
            transform = transformUtils.transformFromPose(pos, quat);
            vis.showFrame(transform, name, parent=folder, scale=0.1);

def init_visualizer():
    frame_viz = FramesVisualizer()

    # Adds to the "Tools" menu.
    applogic.MenuActionToggleHelper(
        'Tools', frame_viz._name,
        frame_viz.is_enabled, frame_viz.set_enabled)
    return frame_viz

# Creates the visualizer when this script is executed.
frame_viz = init_visualizer()
