# Note that this script runs in the main context of drake-visulizer,
# where many modules and variables already exist in the global scope.
from director import lcmUtils
from director import applogic
from director import objectmodel as om
from director import visualization as vis
from director.debugVis import DebugData
import numpy as np
import drake as lcmdrakemsg


class ContactVisualizer(object):

    def __init__(self):
        self._folder_name = 'Contact Results'
        self._name = "Contact Visualizer"
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

        # A map from pair of body names to a list of contact forces
        collision_pair_to_forces = {}
        for contact in msg.contact_info:
            point = np.array([contact.contact_point[0],
                              contact.contact_point[1],
                              contact.contact_point[2]])
            force = np.array([contact.contact_force[0],
                              contact.contact_force[1],
                              contact.contact_force[2]])
            mag = np.linalg.norm(force)
            if mag > 1e-4:
                mag = 0.3 / mag

            key1 = (str(contact.body1_name), str(contact.body2_name))
            key2 = (str(contact.body2_name), str(contact.body1_name))

            if key1 in collision_pair_to_forces:
                collision_pair_to_forces[key1].append(
                    (point, point + mag * force))
            elif key2 in collision_pair_to_forces:
                collision_pair_to_forces[key2].append(
                    (point, point + mag * force))
            else:
                collision_pair_to_forces[key1] = [(point, point + mag * force)]

        for key, list_of_forces in collision_pair_to_forces.iteritems():
            d = DebugData()
            for force_pair in list_of_forces:
                d.addArrow(start=force_pair[0],
                           end=force_pair[1],
                           tubeRadius=0.005,
                           headRadius=0.01)

            vis.showPolyData(
                d.getPolyData(), str(key), parent=folder, color=[0, 1, 0])


def init_visualizer():
    # Create a visualizer instance.
    my_visualizer = ContactVisualizer()

    # Adds to the "Tools" menu.
    applogic.MenuActionToggleHelper(
        'Tools', my_visualizer._name,
        my_visualizer.is_enabled, my_visualizer.set_enabled)
    return my_visualizer


# Creates the visualizer when this script is executed.
contact_viz = init_visualizer()
