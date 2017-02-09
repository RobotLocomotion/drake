# note, this script runs in the main context of drake-visulizer
# where many modules and variables already exist in the global scope

from director import lcmUtils
from director import applogic
import numpy as np
import drake as lcmdrakemsg


class ContactVisualizer(object):
    def __init__(self):
        self.folder_name = 'Contact Results'
        self.name = "Contact Visualizer"
        self.enabled = True
        # Subscribes to the LCM topic
        lcmUtils.addSubscriber(
            'CONTACT_RESULTS',
            messageClass=lcmdrakemsg.lcmt_contact_results_for_viz,
            callback=self.handle_message)

    def is_enabled(self):
        return self.enabled

    def set_enabled(self, enable):
        self.enabled = enable

    def handle_message(self, msg):
        # Removes the folder completely.
        # om is a magic object from director
        om.removeFromObjectModel(om.findObjectByName(self.folder_name))

        # Recreates folder.
        folder = om.getOrCreateContainer(self.folder_name)

        # Doesn't draw anything if disabled.
        if not self.enabled:
            return

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
                    (point, point + mag*force))
            elif key2 in collision_pair_to_forces:
                collision_pair_to_forces[key2].append(
                    (point, point + mag*force))
            else:
                collision_pair_to_forces[key1] = [(point, point + mag*force)]

        for key, list_of_forces in collision_pair_to_forces.iteritems():
            # DebugData is an object from director that provides drawing
            # utilities for primitive geometries.
            d = DebugData()
            for force_pair in list_of_forces:
                d.addArrow(start=force_pair[0],
                           end=force_pair[1],
                           tubeRadius=0.005,
                           headRadius=0.01)

            # vis is an object from director
            vis.showPolyData(
                d.getPolyData(), str(key), parent=folder, color=[0, 1, 0])


def init_visualizer():
    # create visualizer instance
    my_visualizer = ContactVisualizer()

    # add to tools menu
    applogic.MenuActionToggleHelper(
        'Tools', my_visualizer.name,
        my_visualizer.is_enabled, my_visualizer.set_enabled)
    return my_visualizer

# create visualizer when this script is executed
contact_viz = init_visualizer()
