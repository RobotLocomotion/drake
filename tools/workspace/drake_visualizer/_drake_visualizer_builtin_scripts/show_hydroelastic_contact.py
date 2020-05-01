# Note that this script runs in the main context of drake-visualizer,
# where many modules and variables already exist in the global scope.
from director import lcmUtils
from director import applogic
from director import objectmodel as om
from director import visualization as vis
from director.debugVis import DebugData
import director.vtkAll as vtk
import numpy as np
from PythonQt import QtCore, QtGui

import drake as lcmdrakemsg

from _drake_visualizer_builtin_scripts import scoped_singleton_func
from _drake_visualizer_builtin_scripts.show_point_pair_contact \
     import ContactVisModes

# TODO(seancurtis-TRI) Make the dialog box for scaling force arrows in
# show_point_pair_contact.py accessible to this plugin too.

# TODO(seancurtis-TRI) Convert the modal dialog to a mode-less dialog to allow
# continual tweaking of the visualization.


class ColorMapModes:
    '''Common specification of color map modes'''
    @staticmethod
    def get_mode_string(mode):
        if mode == ColorMapModes.kFlameMap:
            return 'Flame'
        elif mode == ColorMapModes.kTwoToneMap:
            return 'Two-tone'
        elif mode == ColorMapModes.kIntensityMap:
            return 'Intensity'
        else:
            return 'Unrecognized mode'

    @staticmethod
    def get_modes():
        return (ColorMapModes.kFlameMap, ColorMapModes.kTwoToneMap,
                ColorMapModes.kIntensityMap)

    @staticmethod
    def get_mode_docstring(mode):
        if mode == ColorMapModes.kFlameMap:
            return 'Color map that maps [min_val, max_val] -> black, blue, '
            'magenta, orange, yellow, white linearly. Saturates to black and '
            'white for values below min_val or above max_val, respectively.'
        elif mode == ColorMapModes.kTwoToneMap:
            return 'Maps [min_val, max_val] to a range of hues between color1 '
            'and color2. Saturates to color1 and color2 for values '
            'below min_val or above max_val, respectively. Presently,'
            'color1 and color2 are blue and hot pink.'
        elif mode == ColorMapModes.kIntensityMap:
            return 'Maps [min_val, max_val] to some color. Saturates to black '
            'and that color for values below min_val or above max_val, '
            'respectively. Presently, the color is red.'
        else:
            return 'unrecognized mode'

    kFlameMap = 0
    kTwoToneMap = 1
    kIntensityMap = 2


class _ColorMapConfigurationDialog(QtGui.QDialog):
    '''A simple dialog for configuring the hydroelastic visualization'''
    def __init__(self, visualizer, show_contact_edges_state,
                 show_pressure_state, show_spatial_force_state,
                 show_traction_vectors_state, show_slip_velocity_vectors_state,
                 max_pressure_observed, reset_max_pressure_observed_functor,
                 parent=None):
        QtGui.QDialog.__init__(self, parent)
        self.setWindowTitle('Hydroelastic contact visualization settings')
        self.reset_max_pressure_observed_functor = \
            reset_max_pressure_observed_functor
        layout = QtGui.QGridLayout()
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)

        row = 0

        # Color map selection.
        layout.addWidget(QtGui.QLabel('Color map'), row, 0)
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
        layout.addWidget(QtGui.QLabel('Minimum pressure'), row, 0)
        self.min_pressure = QtGui.QLineEdit()
        self.min_pressure.setToolTip('Pressures at or less than this value '
                                     'will be visualized as the color defined'
                                     ' at the minimum value of the color map '
                                     '(must be at least zero).')
        self.min_pressure_validator = QtGui.QDoubleValidator(0, 1e20, 2,
                                                             self.min_pressure)
        self.min_pressure_validator.setNotation(
            QtGui.QDoubleValidator.ScientificNotation)
        self.min_pressure.setValidator(self.min_pressure_validator)
        self.min_pressure.setText('{:.3g}'.format(visualizer.min_pressure))
        # TODO(seancurtis-TRI) This is supposed to automatically update max
        # pressure. However, changing min pressure to be larger and then
        # tabbing out of the widget doesn't necessarily send the
        # editingFinished signal (whether it is sent appears to be arbitrary).
        # We need to figure this out before we make a modeless configuration
        # panel.
        self.min_pressure.editingFinished.connect(self.update_max_validator)
        layout.addWidget(self.min_pressure, row, 1)
        row += 1

        # Maximum pressure.
        layout.addWidget(QtGui.QLabel('Maximum pressure'), row, 0)
        self.max_pressure = QtGui.QLineEdit()
        self.max_pressure.setToolTip('Pressures at or greater than this value '
                                     'will be visualized as the color defined'
                                     ' at the maximum value of the color map.')
        self.max_pressure_validator = QtGui.QDoubleValidator(
            0, 1e20, 2, self.max_pressure)
        self.max_pressure_validator.setNotation(
            QtGui.QDoubleValidator.ScientificNotation)
        self.max_pressure.setValidator(self.max_pressure_validator)
        self.max_pressure.setText('{:.3g}'.format(visualizer.max_pressure))
        self.max_pressure.editingFinished.connect(self.update_min_validator)
        layout.addWidget(self.max_pressure, row, 1)
        row += 1

        # Whether to show pressure.
        layout.addWidget(QtGui.QLabel('Render contact surface with pressure'),
                         row, 0)
        self.show_pressure = QtGui.QCheckBox()
        self.show_pressure.setChecked(show_pressure_state)
        self.show_pressure.setToolTip('Renders filled-in polygons with '
                                      'interior coloring representing '
                                      'pressure using the given color map.')
        layout.addWidget(self.show_pressure, row, 1)
        row += 1

        # Whether to show the contact surface as a wireframe.
        layout.addWidget(QtGui.QLabel('Render contact surface wireframe'),
                         row, 0)
        self.show_contact_edges = QtGui.QCheckBox()
        self.show_contact_edges.setChecked(show_contact_edges_state)
        self.show_contact_edges.setToolTip('Renders the edges of the '
                                           'contact surface.')
        layout.addWidget(self.show_contact_edges, row, 1)
        row += 1

        # Whether to show the force and moment vectors.
        layout.addWidget(QtGui.QLabel('Render contact forces and moments'),
                         row, 0)
        self.show_spatial_force = QtGui.QCheckBox()
        self.show_spatial_force.setChecked(show_spatial_force_state)
        self.show_spatial_force.setToolTip('Renders the contact forces (in '
                                           'red) and moments (in blue)')
        layout.addWidget(self.show_spatial_force, row, 1)
        row += 1

        # Whether to show the per-quadrature-point traction vectors.
        layout.addWidget(QtGui.QLabel('Render traction vectors'),
                         row, 0)
        self.show_traction_vectors = QtGui.QCheckBox()
        self.show_traction_vectors.setChecked(show_traction_vectors_state)
        self.show_traction_vectors.setToolTip('Renders the traction vectors '
                                              '(per quadrature point) in '
                                              'magenta')
        layout.addWidget(self.show_traction_vectors, row, 1)
        row += 1

        # Whether to show the per-quadrature-point slip velocity vectors.
        layout.addWidget(QtGui.QLabel('Render slip velocity vectors'),
                         row, 0)
        self.show_slip_velocity_vectors = QtGui.QCheckBox()
        self.show_slip_velocity_vectors.setChecked(
            show_slip_velocity_vectors_state)
        self.show_slip_velocity_vectors.setToolTip('Renders the slip velocity '
                                                   'vectors (per quadrature '
                                                   'point) in cyan')
        layout.addWidget(self.show_slip_velocity_vectors, row, 1)
        row += 1

        # The maximum pressure value recorded and a button to reset it.
        self.pressure_value_label = QtGui.QLabel(
            'Maximum pressure value observed: {:.5e}'.format(
                max_pressure_observed))
        layout.addWidget(self.pressure_value_label, row, 0)
        reset_button = QtGui.QPushButton('Reset max observed pressure')
        reset_button.connect('clicked()',
                             self.reset_max_pressure_observed)
        layout.addWidget(reset_button, row, 1)
        row += 1

        # Accept/cancel.
        btns = QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel
        buttons = QtGui.QDialogButtonBox(btns, QtCore.Qt.Horizontal, self)
        buttons.connect('accepted()', self.accept)
        buttons.connect('rejected()', self.reject)
        layout.addWidget(buttons, row, 0, 1, 2)

        self.setLayout(layout)

    def update_max_validator(self):
        if float(self.max_pressure.text) < float(self.min_pressure.text):
            self.max_pressure.setText(self.min_pressure.text)

    def update_min_validator(self):
        if float(self.min_pressure.text) > float(self.max_pressure.text):
            self.min_pressure.setText(self.max_pressure.text)

    def reset_max_pressure_observed(self):
        self.reset_max_pressure_observed_functor()
        self.pressure_value_label.setText('Maximum pressure value observed: '
                                          '{:.5e}'.format(0))


class ColorMap:
    # Virtual class for mapping a range of values to colors.
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
        return self._do_get_contrasting_color()

    def _do_get_contrasting_color(self):
        raise NotImplementedError('Subclasses need to implement this')

    def _do_get_color(self, norm_value):
        raise NotImplementedError('Subclasses need to implement this')

    def _normalize(self, data, range):
        '''Returns an affine mapped version of the data based on the data range
         provided'''
        (min_val, max_val) = range
        if (min_val > max_val):
            raise AttributeError(
                'Bad range: [{}, {}]'.format(min_val, max_val))
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

    def _do_get_contrasting_color(self):
        # Mint green.
        return [0.6, 1.0, 0.6]


class IntensityMap(ColorMap):
    def __init__(self, data_range=None):
        ColorMap.__init__(self, data_range)

    def _do_get_color(self, norm_data):
        # TODO(drum) Make the color configurable.
        return np.array((1.0, 0.0, 0.0), dtype=np.float) * norm_data

    # TODO(drum): Make this vary when the color is configurable.
    def _do_get_contrasting_color(self):
        return [0.0, 0.0, 1.0]


class TwoToneMap(ColorMap):
    def __init__(self, data_range=None):
        ColorMap.__init__(self, data_range)
        # TODO(drum) Make the two colors configurable. Currently they lie
        # between blue and hot pink.
        self.min_color = np.array((240, 1, 1.0))
        self.max_color = np.array((320.0, 1, 1.0))
        self.delta = self.max_color - self.min_color

    # TODO(drum): Make this vary when the color is configurable.
    def _do_get_contrasting_color(self):
        return [1.0, 1.0, 1.0]

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
        self._name = 'Hydroelastic Contact Visualizer'
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
        self.show_contact_edges = True
        self.show_pressure = True
        self.max_pressure_observed = 0
        self.show_spatial_force = True
        self.show_traction_vectors = False
        self.show_slip_velocity_vectors = False
        # TODO(SeanCurtis-TRI): Make these variables settable through a dialog
        #  box.
        self.magnitude_mode = ContactVisModes.kFixedLength
        self.global_scale = 0.3
        self.min_magnitude = 1e-4

        menu_bar = applogic.getMainWindow().menuBar()
        plugin_menu = get_sub_menu_or_make(menu_bar, '&Plugins')
        contact_menu = get_sub_menu_or_make(plugin_menu, '&Contacts')
        self.configure_action = contact_menu.addAction(
            'Configure &Hydroelastic Contact Visualization')
        self.configure_action.connect('triggered()', self.configure_via_dialog)
        self.set_enabled(True)

    def reset_max_pressure_observed(self):
        self.max_pressure_observed = 0

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
                                           self.show_contact_edges,
                                           self.show_pressure,
                                           self.show_spatial_force,
                                           self.show_traction_vectors,
                                           self.show_slip_velocity_vectors,
                                           self.max_pressure_observed,
                                           self.reset_max_pressure_observed)
        if dlg.exec_() == QtGui.QDialog.Accepted:
            # TODO(edrumwri): Cause this to redraw any pressures that are
            #  currently visualized.
            self.color_map_mode = dlg.color_map_mode.currentIndex
            self.min_pressure = float(dlg.min_pressure.text)
            self.max_pressure = float(dlg.max_pressure.text)
            self.show_contact_edges = dlg.show_contact_edges.isChecked()
            self.show_spatial_force = dlg.show_spatial_force.isChecked()
            self.show_pressure = dlg.show_pressure.isChecked()
            self.show_slip_velocity_vectors =\
                dlg.show_slip_velocity_vectors.isChecked()
            self.show_traction_vectors = dlg.show_traction_vectors.isChecked()

    def add_subscriber(self):
        if self._sub is not None:
            return

        self._sub = lcmUtils.addSubscriber(
            'CONTACT_RESULTS',
            messageClass=lcmdrakemsg.lcmt_contact_results_for_viz,
            callback=self.handle_message)
        print(self._name + ' subscriber added.')

    def remove_subscriber(self):
        if self._sub is None:
            return

        lcmUtils.removeSubscriber(self._sub)
        self._sub = None
        om.removeFromObjectModel(om.findObjectByName(self._folder_name))
        print(self._name + ' subscriber removed.')

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

        # The scale value attributable to auto-scale.
        auto_force_scale = 1.0
        auto_moment_scale = 1.0
        auto_traction_scale = 1.0
        auto_slip_velocity_scale = 1.0
        max_force = -1
        max_moment = -1
        max_traction = -1
        max_slip = -1

        # TODO(sean-curtis-TRI) Remove the following comment when this
        # code can be exercised.
        # The following code is not exercised presently because the
        # magnitude mode is always set to kFixedLength.
        # Determine scaling magnitudes if autoscaling is activated.
        if self.magnitude_mode == ContactVisModes.kAutoScale:
            if self.show_spatial_force:
                for surface in msg.hydroelastic_contacts:
                    force = np.array([surface.force_C_W[0],
                                      surface.force_C_W[1],
                                      surface.force_C_W[2]])
                    moment = np.array([surface.moment_C_W[0],
                                       surface.moment_C_W[1],
                                       surface.moment_C_W[2]])
                    force_mag = np.linalg.norm(force)
                    moment_mag = np.linalg.norm(moment)
                    if force_mag > max_force:
                        max_force = force_mag
                    if moment_mag > max_moment:
                        max_moment = moment_mag

            # Prepare scaling information for the traction vectors.
            if self.show_traction_vectors:
                for quad_point_data in surface.quadrature_point_data:
                    traction = np.array([quad_point_data.traction_Aq_W[0],
                                         quad_point_data.traction_Aq_W[1],
                                         quad_point_data.traction_Aq_W[2]])
                    max_traction = max(max_traction,
                                       np.linalg.norm(traction))

            # Prepare scaling information for the slip velocity vectors.
            if self.show_slip_velocity_vectors:
                for quad_point_data in surface.quadrature_point_data:
                    slip_speed = np.array([quad_point_data.vt_BqAq_W[0],
                                           quad_point_data.vt_BqAq_W[1],
                                           quad_point_data.vt_BqAq_W[2]])
                    max_slip_speed = max(max_slip_speed,
                                         np.linalg.norm(slip_speed))

            # Compute scaling factors.
            auto_force_scale = 1.0 / max_force
            auto_moment_scale = 1.0 / max_moment
            auto_traction_scale = 1.0 / max_traction
            auto_slip_velocity_scale = 1.0 / max_slip_speed

        # TODO(drum) Consider exiting early if no visualization options are
        # enabled.
        for surface in msg.hydroelastic_contacts:
            # Draw the spatial force.
            if self.show_spatial_force:
                point = np.array([surface.centroid_W[0],
                                  surface.centroid_W[1],
                                  surface.centroid_W[2]])
                force = np.array([surface.force_C_W[0],
                                  surface.force_C_W[1],
                                  surface.force_C_W[2]])
                moment = np.array([surface.moment_C_W[0],
                                   surface.moment_C_W[1],
                                   surface.moment_C_W[2]])
                force_mag = np.linalg.norm(force)
                moment_mag = np.linalg.norm(moment)

                # Draw the force arrow if it's of sufficient magnitude.
                if force_mag > self.min_magnitude:
                    scale = self.global_scale
                    if self.magnitude_mode == ContactVisModes.kFixedLength:
                        # magnitude must be > 0 otherwise this force would be
                        # skipped.
                        scale /= force_mag

                    d.addArrow(start=point,
                               end=point + auto_force_scale * force * scale,
                               tubeRadius=0.005,
                               headRadius=0.01, color=[1, 0, 0])

                # Draw the moment arrow if it's of sufficient magnitude.
                if moment_mag > self.min_magnitude:
                    scale = self.global_scale
                    if self.magnitude_mode == ContactVisModes.kFixedLength:
                        # magnitude must be > 0 otherwise this moment would be
                        # skipped.
                        scale /= moment_mag

                    d.addArrow(start=point,
                               end=point + auto_moment_scale * moment * scale,
                               tubeRadius=0.005,
                               headRadius=0.01, color=[0, 0, 1])

            # Iterate over all quadrature points, drawing traction and slip
            # velocity vectors.
            if self.show_traction_vectors or self.show_slip_velocity_vectors:
                for quad_point_data in surface.quadrature_point_data:
                    origin = np.array([quad_point_data.p_WQ[0],
                                       quad_point_data.p_WQ[1],
                                       quad_point_data.p_WQ[2]])

                    if self.show_traction_vectors:
                        traction = np.array([quad_point_data.traction_Aq_W[0],
                                             quad_point_data.traction_Aq_W[1],
                                             quad_point_data.traction_Aq_W[2]])
                        traction_mag = np.linalg.norm(traction)

                        # Draw the arrow only if it's of sufficient magnitude.
                        if traction_mag > self.min_magnitude:
                            scale = self.global_scale
                            if self.magnitude_mode ==\
                                    ContactVisModes.kFixedLength:
                                # magnitude must be > 0 otherwise this traction
                                #  would be skipped.
                                scale /= traction_mag

                            offset = auto_traction_scale * traction * scale
                            d.addArrow(start=origin, end=origin + offset,
                                       tubeRadius=0.000125,
                                       headRadius=0.00025, color=[1, 0, 1])
                        else:
                            d.addSphere(center=origin,
                                        radius=0.000125,
                                        color=[1, 0, 1])

                    if self.show_slip_velocity_vectors:
                        slip = np.array([quad_point_data.vt_BqAq_W[0],
                                         quad_point_data.vt_BqAq_W[1],
                                         quad_point_data.vt_BqAq_W[2]])
                        slip_mag = np.linalg.norm(slip)

                        # Draw the arrow only if it's of sufficient magnitude.
                        if slip_mag > self.min_magnitude:
                            scale = self.global_scale
                            if self.magnitude_mode ==\
                                    ContactVisModes.kFixedLength:
                                # magnitude must be > 0 otherwise this slip
                                # vector would be skipped.
                                scale /= slip_mag

                            offset = auto_slip_velocity_scale * slip * scale
                            d.addArrow(start=origin, end=origin + offset,
                                       tubeRadius=0.000125,
                                       headRadius=0.00025, color=[0, 1, 1])
                        else:
                            d.addSphere(center=origin,
                                        radius=0.000125,
                                        color=[0, 1, 1])

            # Iterate over all triangles.
            for tri in surface.triangles:
                va = np.array([tri.p_WA[0], tri.p_WA[1], tri.p_WA[2]])
                vb = np.array([tri.p_WB[0], tri.p_WB[1], tri.p_WB[2]])
                vc = np.array([tri.p_WC[0], tri.p_WC[1], tri.p_WC[2]])

                # Save the maximum pressure.
                self.max_pressure_observed = max(self.max_pressure_observed,
                                                 tri.pressure_A)
                self.max_pressure_observed = max(self.max_pressure_observed,
                                                 tri.pressure_B)
                self.max_pressure_observed = max(self.max_pressure_observed,
                                                 tri.pressure_C)

                # TODO(drum) Vertex color interpolation may be insufficiently
                # granular if a single triangle spans too large a range of
                # pressures. Suggested solution is to use a texture map.
                # Get the colors at the vertices.
                color_a = color_map.get_color(tri.pressure_A)
                color_b = color_map.get_color(tri.pressure_B)
                color_c = color_map.get_color(tri.pressure_C)

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
                if self.show_contact_edges:
                    contrasting_color = color_map.get_contrasting_color()
                    d.addPolyLine(points=(va, vb, vc), isClosed=True,
                                  color=contrasting_color)

            item_name = '{}, {}'.format(surface.body1_name, surface.body2_name)
            cls = vis.PolyDataItem
            view = applogic.getCurrentRenderView()
            item = cls(item_name, d.getPolyData(), view)
            om.addToObjectModel(item, folder)
            item.setProperty('Visible', True)
            item.setProperty('Alpha', 1.0)

            # Conditional necessary to keep DrakeVisualizer from spewing
            # messages to the console when the contact surface is empty.
            if len(msg.hydroelastic_contacts) > 0:
                item.colorBy('RGB255')


@scoped_singleton_func
def init_visualizer():
    # Create a visualizer instance.
    my_visualizer = HydroelasticContactVisualizer()
    # Adds to the 'Tools' menu.
    applogic.MenuActionToggleHelper(
        'Tools', my_visualizer._name,
        my_visualizer.is_enabled, my_visualizer.set_enabled)
    return my_visualizer


# Activate the plugin if this script is run directly; store the results to keep
# the plugin objects in scope.
if __name__ == '__main__':
    contact_viz = init_visualizer()
