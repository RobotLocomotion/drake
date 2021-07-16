# Note that this script runs in the main context of drake-visualizer,
# where many modules and variables already exist in the global scope.

from director import lcmUtils
from director import applogic
from director import objectmodel as om
from director import visualization as vis
from director.debugVis import DebugData
import numpy as np
from PythonQt import QtCore, QtGui

import drake as lcmdrakemsg

from _drake_visualizer_builtin_scripts import scoped_singleton_func


# TODO(seancurtis-tri) Refactor this out of show_hydroelastic_contact.py and
#                      show_point_pair_contact.py.
class ContactVisModes:
    '''Common specification of contact visualization modes'''
    @staticmethod
    def get_mode_string(mode):
        if mode == ContactVisModes.kFixedLength:
            return "Fixed Length"
        elif mode == ContactVisModes.kScaled:
            return "Scaled"
        elif mode == ContactVisModes.kAutoScale:
            return "Auto-scale"
        else:
            return "Unrecognized mode"

    @staticmethod
    def get_modes():
        return (ContactVisModes.kFixedLength, ContactVisModes.kScaled,
                ContactVisModes.kAutoScale)

    @staticmethod
    def get_mode_docstring(mode):
        if mode == ContactVisModes.kFixedLength:
            return "force vectors have fixed length equal to global scale"
        elif mode == ContactVisModes.kScaled:
            return "simply scaled by global scale"
        elif mode == ContactVisModes.kAutoScale:
            return "largest force has fixed length and all other "\
                   "forces with proportional length on a per-message basis"
        else:
            return "unrecognized mode"

    kFixedLength = 0
    kScaled = 1
    kAutoScale = 2


class _ContactConfigDialog(QtGui.QDialog):
    '''A simple dialog for configuring the contact visualization'''
    def __init__(self, visualizer, parent=None):
        QtGui.QDialog.__init__(self, parent)
        self.setWindowTitle("Point-Contact Force Vector Visualization")
        layout = QtGui.QGridLayout()
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)

        row = 0

        # Magnitude representation
        layout.addWidget(QtGui.QLabel("Vector scaling mode"), row, 0)
        self.magnitude_mode = QtGui.QComboBox()
        modes = ContactVisModes.get_modes()
        mode_labels = [ContactVisModes.get_mode_string(m) for m in modes]
        self.magnitude_mode.addItems(mode_labels)
        self.magnitude_mode.setCurrentIndex(visualizer.magnitude_mode)
        mode_tool_tip = 'Determines how force magnitude is visualized:\n'
        for m in modes:
            mode_tool_tip += '  - {}: {}\n'.format(
                ContactVisModes.get_mode_string(m),
                ContactVisModes.get_mode_docstring(m))
        self.magnitude_mode.setToolTip(mode_tool_tip)
        layout.addWidget(self.magnitude_mode, row, 1)
        row += 1

        # Global scale.
        layout.addWidget(QtGui.QLabel("Global scale"), row, 0)
        self.global_scale = QtGui.QLineEdit()
        self.global_scale.setToolTip(
            'All visualized forces are multiplied by this scale factor (must '
            'be non-negative)')
        validator = QtGui.QDoubleValidator(0, 100, 3, self.global_scale)
        validator.setNotation(QtGui.QDoubleValidator.StandardNotation)
        self.global_scale.setValidator(validator)
        self.global_scale.setText("{:.3f}".format(visualizer.global_scale))
        layout.addWidget(self.global_scale, row, 1)
        row += 1

        # Magnitude cut-off.
        layout.addWidget(QtGui.QLabel("Minimum force"), row, 0)
        self.min_magnitude = QtGui.QLineEdit()
        self.min_magnitude.setToolTip('Forces with a magnitude less than this '
                                      'value will not be visualized (must be '
                                      '> 1e-10)')
        validator = QtGui.QDoubleValidator(1e-10, 100, 10, self.min_magnitude)
        validator.setNotation(QtGui.QDoubleValidator.StandardNotation)
        self.min_magnitude.setValidator(validator)
        self.min_magnitude.setText("{:.3g}".format(visualizer.min_magnitude))
        layout.addWidget(self.min_magnitude, row, 1)
        row += 1

        # Accept/cancel.
        btns = QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel
        buttons = QtGui.QDialogButtonBox(btns, QtCore.Qt.Horizontal, self)
        buttons.connect('accepted()', self.accept)
        buttons.connect('rejected()', self.reject)
        layout.addWidget(buttons, row, 0, 1, 2)

        self.setLayout(layout)


# TODO(SeanCurtis): This would be better extracted out of *this* plugin
def get_sub_menu_or_make(menu, menu_name):
    for a in menu.actions():
        if a.text == menu_name:
            return a.menu()
    return menu.addMenu(menu_name)


class ContactVisualizer:
    def __init__(self):
        self._folder_name = 'Point Pair Contact Results'
        self._name = "Point Pair Contact Visualizer"
        self._enabled = False
        self._sub = None

        # Visualization parameters
        # TODO(SeanCurtis-TRI): Find some way to persist these settings across
        #  invocations of drake visualizer. Config file, environment settings,
        #  something.
        self.magnitude_mode = ContactVisModes.kFixedLength
        self.global_scale = 0.3
        self.min_magnitude = 1e-4

        menu_bar = applogic.getMainWindow().menuBar()
        plugin_menu = get_sub_menu_or_make(menu_bar, '&Plugins')
        contact_menu = get_sub_menu_or_make(plugin_menu, '&Contacts')
        self.configure_action = contact_menu.addAction(
            "&Configure Force Vector for Point Contacts")
        self.configure_action.connect('triggered()', self.configure_via_dialog)

        self.set_enabled(True)
        self.update_screen_text()

    def configure_via_dialog(self):
        '''Configures the visualization'''
        dlg = _ContactConfigDialog(self)
        if dlg.exec_() == QtGui.QDialog.Accepted:
            # TODO(SeanCurtis-TRI): Cause this to redraw any forces that are
            #  currently visualized.
            self.magnitude_mode = dlg.magnitude_mode.currentIndex
            self.global_scale = float(dlg.global_scale.text)
            self.min_magnitude = float(dlg.min_magnitude.text)
            self.update_screen_text()

    def update_screen_text(self):
        folder = om.getOrCreateContainer(self._folder_name)
        my_text = 'Point contact vector: {}'.format(
            ContactVisModes.get_mode_string(self.magnitude_mode))

        # TODO(SeanCurtis-TRI): Figure out how to anchor this in the bottom-
        #  right corner as opposed to floating in the middle.
        w = applogic.getCurrentRenderView().size.width()
        vis.updateText(my_text, 'contact_text',
                       **{'position': (w/2, 10), 'parent': folder})

    def add_subscriber(self):
        if self._sub is not None:
            return

        self._sub = lcmUtils.addSubscriber(
            'CONTACT_RESULTS',
            messageClass=lcmdrakemsg.lcmt_contact_results_for_viz,
            callback=self.handle_message)
        print(self._name + " subscriber added.")

    def remove_subscriber(self):
        if self._sub is None:
            return

        lcmUtils.removeSubscriber(self._sub)
        self._sub = None
        om.removeFromObjectModel(om.findObjectByName(self._folder_name))
        print(self._name + " subscriber removed.")

    def is_enabled(self):
        return self._enabled

    def set_enabled(self, enable):
        self._enabled = enable
        if enable:
            self.add_subscriber()
            self.configure_action.setEnabled(True)
        else:
            self.remove_subscriber()
            self.configure_action.setEnabled(False)
            # Removes the folder completely.
            om.removeFromObjectModel(om.findObjectByName(self._folder_name))

    def handle_message(self, msg):
        # Limits the rate of message handling, since redrawing is done in the
        # message handler.
        self._sub.setSpeedLimit(30)

        # Removes the folder completely.
        om.removeFromObjectModel(om.findObjectByName(self._folder_name))

        # Recreates folder.
        folder = om.getOrCreateContainer(self._folder_name)

        # The scale value attributable to auto-scale.
        auto_scale = 1.0
        max_force = -1

        # A map from pair of body names to a list of contact forces
        collision_pair_to_forces = {}
        for contact in msg.point_pair_contact_info:
            point = np.array([contact.contact_point[0],
                              contact.contact_point[1],
                              contact.contact_point[2]])
            force = np.array([contact.contact_force[0],
                              contact.contact_force[1],
                              contact.contact_force[2]])
            mag = np.linalg.norm(force)

            if mag < self.min_magnitude:
                continue

            if mag > max_force:
                max_force = mag

            scale = self.global_scale
            if self.magnitude_mode == ContactVisModes.kFixedLength:
                # mag must be > 0 otherwise this force would be skipped.
                scale /= mag
            vis_force = force * scale

            # In point_pair_contact_info, the force is defined as the force
            # *on body B*. That is what's placed in
            # lcmt_point_pair_contact_info_for_viz.
            # However, for two bodies (1, 2), MBP provides no guarantees which
            # body is "body1" and which is "body2". Some contacts could be
            # reported as (1, 2) and some as (2, 1). In order to aggregate all
            # such interactions into a single debug artifact, we need to
            # reconcile the ordering.

            force_format = "Force on {} from {}"
            key = force_format.format(contact.body2_name, contact.body1_name)
            alt_key = force_format.format(contact.body1_name,
                                          contact.body2_name)

            if key in collision_pair_to_forces:
                collision_pair_to_forces[key].append((point, vis_force))
            elif alt_key in collision_pair_to_forces:
                collision_pair_to_forces[alt_key].append((point, -vis_force))
            else:
                collision_pair_to_forces[key] = [(point, vis_force)]

        if self.magnitude_mode == ContactVisModes.kAutoScale:
            auto_scale = 1.0 / max_force

        for key, list_of_forces in collision_pair_to_forces.items():
            d = DebugData()
            for p, v in list_of_forces:
                d.addArrow(start=p,
                           end=p + auto_scale * v,
                           tubeRadius=0.005,
                           headRadius=0.01)

            vis.showPolyData(d.getPolyData(), str(key), parent=folder,
                             color=[0.2, 0.8, 0.2])
        self.update_screen_text()


@scoped_singleton_func
def init_visualizer():
    # Create a visualizer instance.
    my_visualizer = ContactVisualizer()
    # Adds to the "Tools" menu.
    applogic.MenuActionToggleHelper(
        'Tools', my_visualizer._name,
        my_visualizer.is_enabled, my_visualizer.set_enabled)
    return my_visualizer


# Activate the plugin if this script is run directly; store the results to keep
# the plugin objects in scope.
if __name__ == "__main__":
    contact_viz = init_visualizer()
