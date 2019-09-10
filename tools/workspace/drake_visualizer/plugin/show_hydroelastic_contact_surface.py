# Note that this script runs in the main context of drake-visualizer,
# where many modules and variables already exist in the global scope.
from director import lcmUtils
from director import applogic
from director import objectmodel as om
from director import visualization as vis
from director.debugVis import DebugData
import numpy as np
from six import iteritems

import drake as lcmdrakemsg

from drake.tools.workspace.drake_visualizer.plugin import scoped_singleton_func


class HydroelasticContactSurfaceVisualizer(object):
    def __init__(self):
        self._folder_name = 'Hydroelastic Contact Surfaces'
        self._name = "Hydroelastic Contact Surface Visualizer"
        self._enabled = False
        self._sub = None

        self.set_enabled(True)

    def add_subscriber(self):
        if self._sub is not None:
            return

        self._sub = lcmUtils.addSubscriber(
            'CONTACT_RESULTS',
            messageClass=lcmdrakemsg.lcmt_contact_results_for_viz,
            callback=self.handle_message)
        print self._name + " subscriber added."

    def remove_subscriber(self):
        if self._sub is None:
            return

        lcmUtils.removeSubscriber(self._sub)
        self._sub = None
        om.removeFromObjectModel(om.findObjectByName(self._folder_name))
        print self._name + " subscriber removed."

    def is_enabled(self):
        return self._enabled

    def set_enabled(self, enable):
        self._enabled = enable
        if enable:
            self.add_subscriber()
        else:
            self.remove_subscriber()

    def handle_message(self, msg):
        # Limits the rate of message handling, since redrawing is done in the
        # message handler.
        self._sub.setSpeedLimit(30)

        # Removes the folder completely.
        om.removeFromObjectModel(om.findObjectByName(self._folder_name))

        # Recreates folder.
        folder = om.getOrCreateContainer(self._folder_name)

        # Though strangely named, DebugData() is the object through which
        # drawing is done in DrakeVisualizer.
        d = DebugData()

        # Iterate over all triangles, drawing the contact surface as a triangle
        # outlined in blue.
        for surface in msg.hydroelastic_contacts:
            for tri in surface.triangles:
                va = np.array([tri.p_WA[0], tri.p_WA[1], tri.p_WA[2]])
                vb = np.array([tri.p_WB[0], tri.p_WB[1], tri.p_WB[2]])
                vc = np.array([tri.p_WC[0], tri.p_WC[1], tri.p_WC[2]])
                d.addLine(p1=va, p2=vb, radius=0, color=[0, 0, 1])
                d.addLine(p1=vb, p2=vc, radius=0, color=[0, 0, 1])
                d.addLine(p1=va, p2=vc, radius=0, color=[0, 0, 1])

            key = (str(surface.body1_name), str(surface.body2_name))
            vis.showPolyData(
                d.getPolyData(), str(key), parent=folder, color=[0, 0, 1])


@scoped_singleton_func
def init_visualizer():
    # Create a visualizer instance.
    my_visualizer = HydroelasticContactSurfaceVisualizer()
    # Adds to the "Tools" menu.
    applogic.MenuActionToggleHelper(
        'Tools', my_visualizer._name,
        my_visualizer.is_enabled, my_visualizer.set_enabled)
    return my_visualizer


# Activate the plugin if this script is run directly; store the results to keep
# the plugin objects in scope.
if __name__ == "__main__":
    contact_viz = init_visualizer()
