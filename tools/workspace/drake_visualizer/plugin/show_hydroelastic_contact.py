# Note that this script runs in the main context of drake-visualizer,
# where many modules and variables already exist in the global scope.
from director import lcmUtils
from director import applogic
from director import objectmodel as om
from director import visualization as vis
from director.debugVis import DebugData
import director.vtkAll as vtk
import numpy as np
from six import iteritems
from PythonQt import QtCore, QtGui

import drake as lcmdrakemsg

from drake.tools.workspace.drake_visualizer.plugin import scoped_singleton_func


class ColorMapModes:
    '''Common specification of color map modes'''
    @staticmethod
    def get_mode_string(mode):
        if mode == ColorMapModes.kFlameMap:
            return "Flame"
        elif mode == ColorMapModes.kTwoToneMap:
            return "Two-tone"
        elif mode == ColorMapModes.kIntensityMap:
            return "Intensity"
        else:
            return "Unrecognized mode"

    @staticmethod
    def get_modes():
        return (ColorMapModes.kFlameMap, ColorMapModes.kTwoToneMap,
                ColorMapModes.kIntensityMap)

    @staticmethod
    def get_mode_docstring(mode):
        if mode == ColorMapModes.kFlameMap:
            return "Color map that maps [min_val, max_val] -> black, blue, "\
                   "magenta, orange, yellow, white linearly. Saturates to "\
                   "black and white for values below min_val or above "\
                   "max_val, respectively"
        elif mode == ColorMapModes.kTwoToneMap:
            return "simply scaled by global scale"
        elif mode == ColorMapModes.kIntensityMap:
            return "largest force has fixed length and all other "\
                   "forces with proportional length on a per-message basis"
        else:
            return "unrecognized mode"

    kFlameMap = 0
    kTwoToneMap = 1
    kIntensityMap = 2


class _ColorMapConfigurationDialog(QtGui.QDialog):
    '''A simple dialog for configuring the hydroelastic
       visualization'''
    def __init__(self, visualizer, show_contact_surface_state,
                 show_pressure_state, parent=None):
        QtGui.QDialog.__init__(self, parent)
        self.setWindowTitle("Hydroelastic contact visualization settings")
        layout = QtGui.QGridLayout()
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)

        row = 0

        # Color map selection.
        layout.addWidget(QtGui.QLabel("Color map"), row, 0)
        self.color_map_mode = QtGui.QComboBox()
        modes = ColorMapModes.get_modes()
        mode_labels = [ColorMapModes.get_mode_string(m) for m in modes]
        self.color_map_mode.addItems(mode_labels)
        self.color_map_mode.setCurrentIndex(visualizer.color_map_mode)
        mode_tool_tip = 'Determines the mapping from pressures to colors:\n'
        for m in modes:
            mode_tool_tip += '  - {}: {}\n'.format(
                ColorMapModes.get_mode_string(m),
                ColorMapModes.get_mode_docstring(m))
        self.color_map_mode.setToolTip(mode_tool_tip)
        layout.addWidget(self.color_map_mode, row, 1)
        row += 1

        # Minimum pressure.
        layout.addWidget(QtGui.QLabel("Minimum pressure"), row, 0)
        self.min_pressure = QtGui.QLineEdit()
        self.min_pressure.setToolTip('Pressures at or less than this value '
                                     'will be visualized as the color defined'
                                     ' at the minimum value of the color map '
                                     '(must be at least zero).')
        validator = QtGui.QDoubleValidator(0, 1e20, 2, self.min_pressure)
        validator.setNotation(QtGui.QDoubleValidator.StandardNotation)
        self.min_pressure.setValidator(validator)
        self.min_pressure.setText("{:.3g}".format(visualizer.min_pressure))
        layout.addWidget(self.min_pressure, row, 1)
        row += 1

        # Maximum pressure.
        layout.addWidget(QtGui.QLabel("Maximum pressure"), row, 0)
        self.max_pressure = QtGui.QLineEdit()
        self.max_pressure.setToolTip('Pressures at or greater than this value '
                                     'will be visualized as the color defined'
                                     ' at the maximum value of the color map '
                                     '(must be larger than the minimum '
                                     'pressure).')
        validator = QtGui.QDoubleValidator(float(self.min_pressure.text),
                                           1e20, 2, self.max_pressure)
        validator.setNotation(QtGui.QDoubleValidator.StandardNotation)
        self.max_pressure.setValidator(validator)
        self.max_pressure.setText("{:.3g}".format(visualizer.max_pressure))
        layout.addWidget(self.max_pressure, row, 1)
        row += 1

        # Whether to show pressure.
        layout.addWidget(QtGui.QLabel("Render contact surface with pressure"),
                         row, 0)
        self.show_pressure = QtGui.QCheckBox()
        if show_pressure_state:
            self.show_pressure.setCheckState(2)
        else:
            self.show_pressure.setCheckState(0)
        self.show_pressure.setToolTip('Blah')
        layout.addWidget(self.show_pressure, row, 1)
        row += 1

        # Whether to show the contact surface as a wireframe.
        layout.addWidget(QtGui.QLabel("Render contact surface wireframe"),
                         row, 0)
        self.show_contact_surface = QtGui.QCheckBox()
        if show_contact_surface_state:
            self.show_contact_surface.setCheckState(2)
        else:
            self.show_contact_surface.setCheckState(0)
        self.show_contact_surface.setToolTip('Blah')
        layout.addWidget(self.show_contact_surface, row, 1)
        row += 1

        # Accept/cancel.
        btns = QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel
        buttons = QtGui.QDialogButtonBox(btns, QtCore.Qt.Horizontal, self)
        buttons.connect('accepted()', self.accept)
        buttons.connect('rejected()', self.reject)
        layout.addWidget(buttons, row, 0, 1, 2)

        self.setLayout(layout)


class ColorMap:
    '''Color map that maps
    [min_val, max_val] -> black, blue, magenta, orange, yellow, white
    linearly. Saturates to black and white for values below min_val or above
    max_val, respectively'''
    def __init__(self, data_range=None):
        if data_range is None:
            self.data_range = [0.0, 10]
        else:
            self.data_range = data_range

    def get_color(self, value, range=None):
        if range is None:
            range = self.data_range
        norm_data = self._normalize(value, range)
        return self._do_get_color(norm_data)

    # Gets a color that will always contrast with those produced by the color
    # map.
    def get_contrasting_color(self):
        return _do_get_contrasting_color()

    def _do_get_contrasting_color(self):
        raise NotImplementedError("Subclasses need to implement this")

    def _do_get_color(self, norm_value):
        raise NotImplementedError("Subclasses need to implement this")

    def _normalize(self, data, (min_val, max_val)):
        """Returns an affine mapped version of the data based on the data range
         provided"""
        if (min_val > max_val):
            raise AttributeError(
                "Bad range: [{}, {}]".format(min_val, max_val))
        assert(max_val >= min_val)
        range = max_val - min_val
        if (range > 0.00001):
            return np.clip((data - min_val) / (max_val - min_val), 0.0, 1.0)
        else:
            return np.zeros_like(data)


class FlameMap(ColorMap):
    '''Color map that maps
    [min_val, max_val] -> black, blue, magenta, orange, yellow, white
    linearly. Saturates to black and white for values below min_val or above
    max_val, respectively'''
    def _do_get_color(self, norm_data):
        color = [0, 0, 0]
        color[0] = np.clip(((norm_data - 0.25) * 4.0), 0.0, 1.0)
        color[1] = np.clip(((norm_data - 0.5) * 4.0), 0.0, 1.0)
        if norm_data < 0.25:
            color[2] = np.clip(norm_data * 4.0, 0.0, 1.0)
        elif norm_data > 0.75:
            color[2] = np.clip((norm_data - 0.75) * 4.0, 0.0, 1.0)
        else:
            color[2] = np.clip(1.0 - (norm_data - 0.25) * 4.0, 0.0, 1.0)
        return color

    def _do_get_contrasting_color:
        # Mint green.
        return [0.6, 1.0, 0.6]


class IntensityMap(ColorMap):
    def _do_get_color(self, norm_data):
        # TODO(drum) Make the color configurable.
        return np.array((0.0, 1.0, 0.0), dtype=np.float) * norm_data

    # TODO(drum): Make this vary when the color is configurable.
    def _do_get_contrasting_color:
        return [1.0, 0.0, 0.0]


class TwoToneMap(ColorMap):
    def __init__(self):
        ColorMap.__init__(self)
        # TODO(drum) Make the two colors configurable. Currently they lie
        # between hot pink and blue.
        self.min_color = np.array((240, 1, 1.0))
        self.max_color = np.array((320.0, 1, 1.0))
        self.delta = self.max_color - self.min_color

    # TODO(drum): Make this vary when the color is configurable.
    def _do_get_contrasting_color:
        return [0.0, 0.0, 1.0]

    def _do_get_color(self, norm_data):
        hsv = self.min_color + self.delta * norm_data
        return self.hsvToRgb(hsv)

    def hsvToRgb(self, hsv):
        '''Convert hue, saturation and lightness to r, g, b values.
        Hue in [0, 360], s in [0, 1], l in [0, 1].'''
        h, s, v = hsv
        r = g = b = 0.0
        c = s * v
        h /= 60.0
        x = c * (1 - abs((h % 2) - 1))

        if (h >= 0 and h < 1):
            r = c
            g = x
        elif (h >= 1 and h < 2):
            r = x
            g = c
        elif (h >= 2 and h < 3):
            g = c
            b = x
        elif (h >= 3 and h < 4):
            g = x
            b = c
        elif (h >= 4 and h < 5):
            r = x
            b = c
        else:
            r = c
            b = x
        m = v - c
        r += m
        g += m
        b += m
        return r, g, b


# TODO(SeanCurtis): This would be better extracted out of *this* plugin
def get_sub_menu_or_make(menu, menu_name):
    for a in menu.actions():
        if a.text == menu_name:
            return a.menu()
    return menu.addMenu(menu_name)


class HydroelasticContactVisualizer(object):
    def __init__(self):
        self._folder_name = 'Hydroelastic Contact'
        self._name = "Hydroelastic Contact Visualizer"
        self._enabled = False
        self._sub = None

        self.set_enabled(True)

        # Visualization parameters
        # TODO(SeanCurtis-TRI): Find some way to persist these settings across
        #  invocations of drake visualizer. Config file, environment settings,
        #  something.
        self.color_map_mode = ColorMapModes.kFlameMap
        self.min_pressure = 0
        self.max_pressure = 10
        self.show_contact_surface = True
        self.show_pressure = True

        menu_bar = applogic.getMainWindow().menuBar()
        plugin_menu = get_sub_menu_or_make(menu_bar, '&Plugins')
        contact_menu = get_sub_menu_or_make(plugin_menu, '&Contacts')
        self.configure_action = contact_menu.addAction(
            "Configure &Hydroelastic Contact Visualization")
        self.configure_action.connect('triggered()', self.configure_via_dialog)
        self.set_enabled(True)

    def create_color_map(self):
        if self.color_map_mode == ColorMapModes.kFlameMap:
            return FlameMap([self.min_pressure, self.max_pressure])
        if self.color_map_mode == ColorMapModes.kIntensityMap:
            return IntensityMap([self.min_pressure, self.max_pressure])
        if self.color_map_mode == ColorMapModes.kTwoToneMap:
            return TwoToneMap([self.min_pressure, self.max_pressure])
        # Should never be here.
        assert False

    def configure_via_dialog(self):
        '''Configures the visualization'''
        dlg = _ColorMapConfigurationDialog(self,
                                           self.show_contact_surface,
                                           self.show_pressure)
        if dlg.exec_() == QtGui.QDialog.Accepted:
            # TODO(edrumwri): Cause this to redraw any pressures that are
            #  currently visualized.
            self.color_map_mode = dlg.color_map_mode.currentIndex
            self.min_pressure = float(dlg.min_pressure.text)
            self.max_pressure = float(dlg.max_pressure.text)
            self.show_contact_surface =\
                (dlg.show_contact_surface.checkState() > 0)
            self.show_pressure = (dlg.show_pressure.checkState() > 0)

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

        # Set the color map.
        color_map = self.create_color_map()

        # TODO(drum) Consider exiting early if no visualization options are
        # enabled.
        # Iterate over all triangles.
        for surface in msg.hydroelastic_contacts:
            # Avoid warnings to console in the (unexpected but allowable) case
            # that the contact surface is empty.
            if len(surface.triangles) == 0:
                continue

            for tri in surface.triangles:
                va = np.array([tri.p_WA[0], tri.p_WA[1], tri.p_WA[2]])
                vb = np.array([tri.p_WB[0], tri.p_WB[1], tri.p_WB[2]])
                vc = np.array([tri.p_WC[0], tri.p_WC[1], tri.p_WC[2]])

                # Get the colors at the vertices.
                color_a = color_map.get_color(tri.pressure_A)
                color_b = color_map.get_color(tri.pressure_B)
                color_c = color_map.get_color(tri.pressure_C)

                # Get the inverted colors.
                inv_color_a = color_map.get_inverted_color(tri.pressure_A)
                inv_color_b = color_map.get_inverted_color(tri.pressure_B)
                inv_color_c = color_map.get_inverted_color(tri.pressure_C)

                if self.show_pressure:
                    # TODO(drum) Use a better method for this; the current
                    # approach is susceptible to z-fighting under certain
                    # zoom levels.

                    # Compute a normal to the triangle. We need this normal
                    # because the visualized pressure surface can be coplanar
                    # with parts of the visualized geometry, in which case a
                    # dithering type effect would appear. So we use the normal
                    # to draw two triangles slightly offset to both sides of
                    # the contact surface.

                    # Note that if the area of this triangle is very small, we
                    # won't waste time visualizing it, which also means that
                    # won't have to worry about degenerate triangles).

                    # TODO(edrumwri) Consider allowing the user to set these
                    # next two values programmatically.
                    min_area = 1e-8
                    offset_scalar = 1e-4
                    normal = np.cross(vb - va, vc - vb)
                    norm_normal = np.linalg.norm(normal)
                    if norm_normal >= min_area:
                        unit_normal = normal / np.linalg.norm(normal)
                        offset = unit_normal * offset_scalar

                        d.addPolygon([va + offset, vb + offset, vc + offset],
                                     color=[color_a, color_b, color_c])
                        d.addPolygon([va - offset, vb - offset, vc - offset],
                                     color=[color_a, color_b, color_c])

                # TODO(drum) Consider drawing shared edges just once.
                if self.show_contact_surface:
                    d.addLine(p1=va, p2=vb, radius=0,
                              color=[inv_color_a, inv_color_b])
                    d.addLine(p1=vb, p2=vc, radius=0,
                              color=[inv_color_b, inv_color_c])
                    d.addLine(p1=va, p2=vc, radius=0,
                              color=[inv_color_a, inv_color_c])

            key = (str(surface.body1_name), str(surface.body2_name))
            cls = vis.PolyDataItem
            view = applogic.getCurrentRenderView()
            item = cls(str(key), d.getPolyData(), view)
            om.addToObjectModel(item, folder)
            item.setProperty('Visible', True)
            item.setProperty('Alpha', 1.0)
            item.colorBy('RGB255')


@scoped_singleton_func
def init_visualizer():
    # Create a visualizer instance.
    my_visualizer = HydroelasticContactVisualizer()
    # Adds to the "Tools" menu.
    applogic.MenuActionToggleHelper(
        'Tools', my_visualizer._name,
        my_visualizer.is_enabled, my_visualizer.set_enabled)
    return my_visualizer


# Activate the plugin if this script is run directly; store the results to keep
# the plugin objects in scope.
if __name__ == "__main__":
    contact_viz = init_visualizer()
