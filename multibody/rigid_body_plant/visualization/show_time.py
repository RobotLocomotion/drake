# Note that this script runs in the main context of drake-visulizer,
# where many modules and variables already exist in the global scope.
from director import lcmUtils
from director import applogic
import bot_core as lcmbotcore
import time

class TimeVisualizer(object):
    def __init__(self):
        self._name = "Time Visualizer"
        self._real_time = [];
        self._msg_time = [];
        self._subscriber = None
        # Number of messages used to average for real time factor.
        self._num_msg_for_average = 50

        self.set_enabled(True)

    def add_subscriber(self):
        if (self._subscriber is not None):
            return

        self._subscriber = lcmUtils.addSubscriber(
            'DRAKE_VIEWER_DRAW',
            messageClass = lcmbotcore.viewer_draw_t,
            callback = self.handle_message)

    def remove_subscriber(self):
        if (self._subscriber is None):
            return

        lcmUtils.removeSubscriber(self._subscriber)
        self._subscriber = None
        vis.updateText('', 'text')

    def is_enabled(self):
        return self._subscriber is not None

    def set_enabled(self, enable):
        if enable:
            self.add_subscriber()
        else:
            self.remove_subscriber()

    def handle_message(self, msg):
        msg_time = msg.timestamp * 1e-3 # convert from milliseconds
        self._real_time.append(time.time())
        self._msg_time.append(msg_time)

        my_text = 'sim time: %.3f' % msg_time

        if (len(self._real_time) >= self._num_msg_for_average):
            self._real_time.pop(0)
            self._msg_time.pop(0)

            dt = self._msg_time[-1] - self._msg_time[0]
            dt_real_time = self._real_time[-1] - self._real_time[0]

            rt_ratio = dt / dt_real_time

            my_text = my_text + ', real time factor: %.2f' % rt_ratio

        vis.updateText(my_text, 'text')

def init_visualizer():
    time_viz = TimeVisualizer()

    # Adds to the "Tools" menu.
    applogic.MenuActionToggleHelper(
        'Tools', time_viz._name,
        time_viz.is_enabled, time_viz.set_enabled)
    return time_viz

# Creates the visualizer when this script is executed.
time_viz = init_visualizer()
