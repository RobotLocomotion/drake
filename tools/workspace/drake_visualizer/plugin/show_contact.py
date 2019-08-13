# Note that this script runs in the main context of drake-visualizer,
# where many modules and variables already exist in the global scope.
from director import lcmUtils
from director import applogic
from director import objectmodel as om
from director import visualization as vis
from director.debugVis import DebugData
import numpy as np
from PythonQt import QtCore, QtGui
from six import iteritems

import drake as lcmdrakemsg

from drake.tools.workspace.drake_visualizer.plugin import scoped_singleton_func


class ContactConfigDialog(QtGui.QDialog):
    '''A simple dialog for configuring the contact visualization'''
    def __init__(self, visualizer, parent=None):
        QtGui.QDialog.__init__(self, parent)
        self.setWindowTitle("Force Vector Visualization")
        layout = QtGui.QGridLayout()
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)

        row = 0

        # Magnitude representation
        layout.addWidget(QtGui.QLabel("Magnitude representation"), row, 0)
        self.magnitude_mode = QtGui.QComboBox()
        self.magnitude_mode.addItems(('Fixed length', 'Scale', 'Auto-scale'))
        self.magnitude_mode.setCurrentIndex(visualizer.magnitude_mode)
        self.magnitude_mode.setToolTip(
            'Determines how force magnitude is visualized:\n'
            '  - fixed length: force vectors have fixed length equal to global'
            ' scale\n'
            '  - scale: simply scaled by global scale\n'
            '  - auto-scale: largest force has fixed length and all other '
            'forces with proportional length on a per-message basis.')
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

    kFixedLength = 0
    kScaled = 1
    kAutoScale = 2


class ContactVisualizer(object):
    def __init__(self):
        self._folder_name = 'Contact Results'
        self._name = "Contact Visualizer"
        self._enabled = False
        self._sub = None

        # Visualization parameters
        self.magnitude_mode = ContactConfigDialog.kFixedLength
        self.global_scale = 0.3
        self.min_magnitude = 1e-4

        main_window = applogic.getMainWindow()
        self.contact_menu = main_window.menuBar().addMenu('&Contacts')
        self.configure_action = self.contact_menu.addAction(
            "&Configure Force Vector")
        self.configure_action.connect('triggered()', self.do_configure)

        self.set_enabled(True)
        self.set_text()

    def do_configure(self):
        '''Configures the visualization'''
        dlg = ContactConfigDialog(self)
        if dlg.exec_() == QtGui.QDialog.Accepted:
            # TODO(SeanCurtis-TRI): Cause this to redraw any forces that are
            #  currently visualized.
            self.magnitude_mode = dlg.magnitude_mode.currentIndex
            self.global_scale = float(dlg.global_scale.text)
            self.min_magnitude = float(dlg.min_magnitude.text)
            self.set_text()

    def set_text(self):
        folder = om.getOrCreateContainer(self._folder_name)
        my_text = 'Contact vector: '
        if self.magnitude_mode == ContactConfigDialog.kFixedLength:
            my_text += 'fixed length'
        elif self.magnitude_mode == ContactConfigDialog.kScaled:
            my_text += 'scaled forces'
        elif self.magnitude_mode == ContactConfigDialog.kAutoScale:
            my_text += 'auto-scale'
        else:
            my_text += "ERROR! Unrecognized magnitude mode"

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
        for contact in msg.contact_info:
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
            if self.magnitude_mode == ContactConfigDialog.kFixedLength:
                # mag must be > 0 otherwise this force would be skipped.
                scale /= mag
            vis_force = force * scale

            key1 = (str(contact.body1_name), str(contact.body2_name))
            key2 = (str(contact.body2_name), str(contact.body1_name))

            if key1 in collision_pair_to_forces:
                collision_pair_to_forces[key1].append(
                    (point, vis_force))
            elif key2 in collision_pair_to_forces:
                collision_pair_to_forces[key2].append(
                    (point, vis_force))
            else:
                collision_pair_to_forces[key1] = [(point, vis_force)]

        if self.magnitude_mode == ContactConfigDialog.kAutoScale:
            auto_scale = 1.0 / max_force

        for key, list_of_forces in iteritems(collision_pair_to_forces):
            d = DebugData()
            for p, v in list_of_forces:
                d.addArrow(start=p,
                           end=p + auto_scale * v,
                           tubeRadius=0.005,
                           headRadius=0.01)

            vis.showPolyData(d.getPolyData(), str(key), parent=folder,
                             color=[0.2, 0.8, 0.2])
        self.set_text()


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
