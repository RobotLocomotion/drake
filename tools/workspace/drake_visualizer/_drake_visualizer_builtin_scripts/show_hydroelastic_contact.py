# Note that this script runs in the main context of drake-visualizer,
# where many modules and variables already exist in the global scope.
from director import lcmUtils
from director import applogic
from director import objectmodel as om
from director import visualization as vis
from director.debugVis import DebugData
import director.vtkAll as vtk
import director.vtkNumpy as vnp
import numpy as np
from PythonQt import QtCore, QtGui

from drake import (
    lcmt_contact_results_for_viz,
    lcmt_hydroelastic_contact_surface_for_viz,
    lcmt_viewer_load_robot,
)

from _drake_visualizer_builtin_scripts import scoped_singleton_func
from _drake_visualizer_builtin_scripts.show_point_pair_contact \
    import ContactVisModes
# TODO(seancurtis-TRI) Make the dialog box for scaling force arrows in
# show_point_pair_contact.py accessible to this plugin too.


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
    def get_modes_and_maps():
        return ((ColorMapModes.kFlameMap, FlameMap()),
                (ColorMapModes.kTwoToneMap, TwoToneMap()),
                (ColorMapModes.kIntensityMap, IntensityMap()))

    @staticmethod
    def get_mode_docstring(mode):
        if mode == ColorMapModes.kFlameMap:
            return ('Color map that maps [min_val, max_val] -> black, blue, '
                    'magenta, orange, yellow, white linearly. Saturates to '
                    'black and white for values below min_val or above '
                    'max_val, respectively.')
        elif mode == ColorMapModes.kTwoToneMap:
            return ('Maps [min_val, max_val] to a range of hues between '
                    'color1 and color2. Saturates to color1 and color2 for '
                    'values below min_val or above max_val, respectively. '
                    'Presently, color1 and color2 are blue and hot pink.')
        elif mode == ColorMapModes.kIntensityMap:
            return ('Maps [min_val, max_val] to some color. Saturates to '
                    'black and that color for values below min_val or above '
                    'max_val, respectively. Presently, the color is red.')
        else:
            return 'unrecognized mode'

    kFlameMap = 0
    kTwoToneMap = 1
    kIntensityMap = 2


def create_color_map_icon(size, color_map):
    """Creates a QIcon representing the color map as a horizontal gradient."""
    color_map.data_range = [0, 1]
    samples = np.linspace(0.0, 1.0, size.width())
    image = QtGui.QImage(size.width(), 1, QtGui.QImage.Format_RGB32)
    for i in range(size.width()):
        [r, g, b] = color_map.get_color(samples[i])
        rgb = QtGui.QRgba64.fromRgba(int(r * 255), int(g * 255),
                                     int(b * 255), 255).toArgb32()
        image.setPixel(i, 0, rgb)
    pixmap = QtGui.QPixmap()
    pixmap.convertFromImage(image.scaled(size))
    return QtGui.QIcon(pixmap)


class _ConfigDialog(QtGui.QDialog):
    '''A simple dialog for configuring the hydroelastic visualization'''

    def __init__(self, visualizer, parent=None):
        QtGui.QDialog.__init__(self, parent)
        self.setModal(False)
        self.setWindowTitle('Hydroelastic contact visualization settings')
        layout = QtGui.QGridLayout()
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)

        row = 0

        # Color map selection.
        layout.addWidget(QtGui.QLabel('Color map'), row, 0)
        self.color_map_mode = QtGui.QComboBox()
        mode_tool_tip = 'Determines the mapping from pressures to colors:\n'
        icon_size = QtCore.QSize(96, 16)
        for mode, color_map in ColorMapModes.get_modes_and_maps():
            label = ColorMapModes.get_mode_string(mode)
            icon = create_color_map_icon(icon_size, color_map)
            self.color_map_mode.addItem(icon, label)
            mode_tool_tip += '  - {}: {}\n'.format(
                ColorMapModes.get_mode_string(mode),
                ColorMapModes.get_mode_docstring(mode))
        self.color_map_mode.setIconSize(icon_size)
        self.color_map_mode.setToolTip(mode_tool_tip)
        self.color_map_mode.setCurrentIndex(visualizer.color_map_mode)
        layout.addWidget(self.color_map_mode, row, 1)
        row += 1

        # Minimum pressure.
        layout.addWidget(QtGui.QLabel('Minimum pressure'), row, 0)
        self.min_pressure = QtGui.QLineEdit()
        self.min_pressure.setToolTip('Pressures at or less than this value '
                                     'will be visualized as the color defined'
                                     ' at the minimum value of the color map '
                                     '(must be at least zero).')
        min_pressure_validator = QtGui.QDoubleValidator(0, 1e20, 2,
                                                        self.min_pressure)
        min_pressure_validator.setNotation(
            QtGui.QDoubleValidator.ScientificNotation)
        self.min_pressure.setValidator(min_pressure_validator)
        self.min_pressure.setText('{:.3g}'.format(visualizer.min_pressure))
        self.min_pressure.editingFinished.connect(self.update_max_validator)
        layout.addWidget(self.min_pressure, row, 1)
        row += 1

        # Maximum pressure.
        layout.addWidget(QtGui.QLabel('Maximum pressure'), row, 0)
        self.max_pressure = QtGui.QLineEdit()
        self.max_pressure.setToolTip('Pressures at or greater than this value '
                                     'will be visualized as the color defined'
                                     ' at the maximum value of the color map.')
        max_pressure_validator = QtGui.QDoubleValidator(
            0, 1e20, 2, self.max_pressure)
        max_pressure_validator.setNotation(
            QtGui.QDoubleValidator.ScientificNotation)
        self.max_pressure.setValidator(max_pressure_validator)
        self.max_pressure.setText('{:.3g}'.format(visualizer.max_pressure))
        self.max_pressure.editingFinished.connect(self.update_min_validator)
        layout.addWidget(self.max_pressure, row, 1)
        row += 1

        # Whether to show pressure.
        layout.addWidget(QtGui.QLabel('Render contact surface with pressure'),
                         row, 0)
        self.show_pressure = QtGui.QCheckBox()
        self.show_pressure.setChecked(visualizer.show_pressure)
        self.show_pressure.setToolTip('Renders filled-in polygons with '
                                      'interior coloring representing '
                                      'pressure using the given color map.')
        layout.addWidget(self.show_pressure, row, 1)
        row += 1

        layout.addWidget(QtGui.QLabel("Render contact surface edges"), row, 0)
        self.show_edges = QtGui.QCheckBox()
        self.show_edges.setChecked(visualizer.show_edges)
        self.show_pressure.setToolTip('Renders the edges of the contact '
                                      'surface faces.')
        layout.addWidget(self.show_edges, row, 1)
        self.show_edges.clicked.connect(self.toggle_show_edges)
        row += 1

        layout.addWidget(QtGui.QLabel("Edge width"), row, 0,
                         QtCore.Qt.AlignRight)
        mini_layout = QtGui.QGridLayout()
        mini_layout.setColumnStretch(0, 0)
        mini_layout.setColumnStretch(1, 1)
        layout.addLayout(mini_layout, row, 1)
        self.edge_width_display = QtGui.QLabel(str(visualizer.edge_width))
        mini_layout.addWidget(self.edge_width_display, 0, 0)
        self.edge_width = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.edge_width.setToolTip('Controls the thickness of the visible '
                                   'contact surface edges. Use the left/right '
                                   'arrow keys or drag on the handle to '
                                   'change width.')
        self.edge_width.value = visualizer.edge_width
        self.edge_width.minimum = 0
        self.edge_width.maximum = 10
        # When a user clicks on the bar and not the handle, this causes the
        # handle to take one step toward the mouse click from the current
        # position. Ideally, the handle should jump straight to the position.
        # This stackoverflow suggests a method:
        # https://stackoverflow.com/questions/11132597/qslider-mouse-direct-jump?rq=1 # noqa
        # however, attempts to implement the solution produced segfaults when
        # invoking the parent class's setStyle() method. So, this is left
        # as an exercise for future readers.
        self.edge_width.setPageStep(1)
        mini_layout.addWidget(self.edge_width, 0, 1)
        self.edge_width.valueChanged.connect(self.update_width_label)
        row += 1

        contact_data_grp = QtGui.QGroupBox("Contact data")
        contact_layout = QtGui.QGridLayout()
        contact_data_grp.setLayout(contact_layout)
        contact_layout.setColumnStretch(0, 0)
        contact_layout.setColumnStretch(1, 1)
        contact_row = 0
        layout.addWidget(contact_data_grp, row, 0, 1, 2)
        row += 1

        # Whether to show the force and moment vectors.
        contact_layout.addWidget(
            QtGui.QLabel('Render contact force and moment '
                         'vectors'),
            contact_row, 0)
        self.show_spatial_force = QtGui.QCheckBox()
        self.show_spatial_force.setChecked(visualizer.show_spatial_force)
        self.show_spatial_force.setToolTip('Renders the contact forces (in '
                                           'red) and moments (in blue)')
        contact_layout.addWidget(self.show_spatial_force, contact_row, 1)
        contact_row += 1

        # Whether to show the per-quadrature-point traction vectors.
        contact_layout.addWidget(QtGui.QLabel('Render traction vectors'),
                                 contact_row, 0)
        self.show_traction_vectors = QtGui.QCheckBox()
        self.show_traction_vectors.setChecked(visualizer.show_traction_vectors)
        self.show_traction_vectors.setToolTip('Renders the traction vectors '
                                              '(per quadrature point) in '
                                              'magenta')
        contact_layout.addWidget(self.show_traction_vectors, contact_row, 1)
        contact_row += 1

        # Whether to show the per-quadrature-point slip velocity vectors.
        contact_layout.addWidget(QtGui.QLabel('Render slip velocity vectors'),
                                 contact_row, 0)
        self.show_slip_velocity_vectors = QtGui.QCheckBox()
        self.show_slip_velocity_vectors.setChecked(
            visualizer.show_slip_velocity_vectors)
        self.show_slip_velocity_vectors.setToolTip('Renders the slip velocity '
                                                   'vectors (per quadrature '
                                                   'point) in cyan')
        contact_layout.addWidget(
            self.show_slip_velocity_vectors, contact_row, 1)
        contact_row += 1

        # TODO(DamrongGuoy): The following three widgets "Magnitude
        #  representation", "Global scale", and "Magnitude cut-off" are copied
        #  and modified from show_point_pair_contact.py _ContactConfigDialog().
        #  We should have both show_hydroelastic_contact.py and
        #  show_point_pair_contact.py share the code instead of duplication.
        #  Furthermore, we should have this setting for each of force, moment,
        #  traction, and slip vectors. See issue #14680.

        # Magnitude representation
        layout.addWidget(QtGui.QLabel("Vector scaling mode"), row, 0)
        self.magnitude_mode = QtGui.QComboBox()
        modes = ContactVisModes.get_modes()
        mode_labels = [ContactVisModes.get_mode_string(m) for m in modes]
        self.magnitude_mode.addItems(mode_labels)
        self.magnitude_mode.setCurrentIndex(visualizer.magnitude_mode)
        mode_tool_tip = ('Determines how the magnitude of all hydroelastic '
                         'vector quantities are visualized:\n')
        for m in modes:
            mode_tool_tip += '  - {}: {}\n'.format(
                ContactVisModes.get_mode_string(m),
                ContactVisModes.get_mode_docstring(m))
        self.magnitude_mode.setToolTip(mode_tool_tip)
        layout.addWidget(self.magnitude_mode, row, 1)
        row += 1

        # Global scale.
        layout.addWidget(QtGui.QLabel("Global scale of all vectors"), row, 0)
        self.global_scale = QtGui.QLineEdit()
        self.global_scale.setToolTip(
            'All visualized vectors are multiplied by this scale factor (must '
            'be non-negative and at most 100). It is dimensionless.')
        validator = QtGui.QDoubleValidator(0, 100, 3, self.global_scale)
        validator.setNotation(QtGui.QDoubleValidator.ScientificNotation)
        self.global_scale.setValidator(validator)
        self.global_scale.setText("{:.3f}".format(visualizer.global_scale))
        layout.addWidget(self.global_scale, row, 1)
        row += 1

        # Magnitude cut-off.
        layout.addWidget(QtGui.QLabel("Minimum vector"), row, 0)
        self.min_magnitude = QtGui.QLineEdit()
        self.min_magnitude.setToolTip('Vectors with a magnitude less than '
                                      'this value will not be visualized '
                                      '(must be > 1e-10 and at most 100')
        validator = QtGui.QDoubleValidator(1e-10, 100, 10, self.min_magnitude)
        validator.setNotation(QtGui.QDoubleValidator.ScientificNotation)
        self.min_magnitude.setValidator(validator)
        self.min_magnitude.setText("{:.3g}".format(visualizer.min_magnitude))
        layout.addWidget(self.min_magnitude, row, 1)
        row += 1

        # The maximum pressure value recorded and a button to reset it.
        self.pressure_value_label = QtGui.QLabel()
        self.set_max_pressure(visualizer.max_pressure_observed)
        layout.addWidget(self.pressure_value_label, row, 0)
        self.reset_button = QtGui.QPushButton('Reset max observed pressure')
        layout.addWidget(self.reset_button, row, 1)
        row += 1

        # Accept/cancel.
        close_btn = QtGui.QPushButton("Close", self)
        close_btn.connect("clicked()", self.accept)
        close_btn.setAutoDefault(False)
        layout.addWidget(close_btn, row, 0, 1, 2)

        self.setLayout(layout)

    def update_max_validator(self):
        if float(self.max_pressure.text) < float(self.min_pressure.text):
            self.max_pressure.setText(self.min_pressure.text)

    def update_min_validator(self):
        if float(self.min_pressure.text) > float(self.max_pressure.text):
            self.min_pressure.setText(self.max_pressure.text)

    def set_max_pressure(self, value):
        """Slot for changing what the observed maximum pressure is."""
        self.pressure_value_label.setText("Maximum pressure value observed: "
                                          f"{value:.5e}")

    def toggle_show_edges(self, state):
        self.edge_width.setEnabled(state)

    def update_width_label(self, value):
        self.edge_width_display.text = str(value)


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

    def _normalize(self, data, range=None):
        '''Returns an affine mapped version of the data based on the data range
         provided'''
        if range is None:
            range = self.data_range
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

    def _high_intensity(self):
        """Returns the color for highest intensity."""
        # For the auto contrasting color, make sure this color is always full
        # contrast (e.g., one channel must be zero).
        return np.array((1.0, 0.6, 0.0), dtype=np.float)

    def _do_get_color(self, norm_data):
        # TODO(drum) Make the color configurable.
        return self._high_intensity() * norm_data

    def _do_get_contrasting_color(self):
        # This works best for full saturation colors because at least one
        # channel of the high intensity color will be zero. That means its
        # contrasting color will likewise be full saturation. However, if the
        # high intensity color were white, this would become black and wouldn't
        # contrast with the low intensity black. We need a color that is
        # reasonably contrasty with both colors simultaneously.
        return np.array((1.0, 1.0, 1.0)) - self._high_intensity()


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


def create_texture(texture_size, color_map):
    """Creates a texture image by uniformly sampling the color space in the
    given color map."""
    color_map.data_range = [0, 1]
    samples = np.linspace(0.0, 1.0, texture_size)
    texture_image = vtk.vtkImageCanvasSource2D()
    texture_image.SetScalarTypeToUnsignedChar()
    texture_image.SetExtent(0, texture_size-1, 0, 0, 0, 0)
    texture_image.SetNumberOfScalarComponents(3)
    for i in range(0, texture_size):
        [r, g, b] = color_map.get_color(samples[i])
        texture_image.SetDrawColor(255*r, 255*g, 255*b, 0)
        texture_image.DrawPoint(i, 0)
    texture_image.Update()
    texture = vtk.vtkTexture()
    texture.SetInputConnection(texture_image.GetOutputPort())
    texture.RepeatOff()
    return texture


class VisualItem:
    """Represents a single object added to the object model to represent an
    aspect of a single contact: e.g., pressure patch, surface mesh, traction
    forces, etc.

    This associates the object model item and arbitrary user data. The user
    data can be used in adding or updating the item via custom callbacks.
    (The pressure mesh makes use of it, for example.)"""
    def __init__(self, item: vis.PolyDataItem):
        """Constructs a VisualItem.

        Args:
            vis_item: the PolyDataItem containing the visual element for
                director's object model.
            user_data: A tuple of objects associated with the item. These
                can be used by the callbacks for updating."""
        self.item = item
        self.user_data = ()


class _Contact:
    """Tracks the contact object model components for a single contact. A
    single contact is represented by zero or more components: e.g., pressure
    field, force vectors, etc.). The items are stored in a map associating item
    name with its corresponding VisualItem."""

    def __init__(self,
                 surface: lcmt_hydroelastic_contact_surface_for_viz,
                 folder: om.ContainerItem,
                 timestamp: int):
        """Creates an empty Contact for the given surface.

        Args:
            surface: The contact surface data to add to the object model.
            folder: The parent container in the model to which to add items.
            timestamp: The creation timestamp."""
        self._key = self.make_key(surface)
        self.folder = folder
        self.timestamp = timestamp
        # Maps the item *label* to the VisualItem containing its data.
        self.items = {}

    @staticmethod
    def make_key(surface: lcmt_hydroelastic_contact_surface_for_viz):
        """Creates the key for this contact surface (based on the
        geometries involved).

        Args:
            surface: The contact surface data."""
        need_names = (surface.collision_count1 > 1
                      or surface.collision_count2 > 1)
        return (surface.geometry1_name, surface.geometry2_name, need_names)

    @staticmethod
    def _contact_label_suffix(key):
        """Creates a contact label based on the contact key."""
        name1, name2, need_names = key
        if need_names:
            return f' ({name1}, {name2})'
        return ""

    def clear(self):
        """Clears all the contact data for this contact."""
        for vis_item in self.items.values():
            om.removeFromObjectModel(vis_item.item)
        self.items = {}

    def update_item(self, item_name: str, callback):
        """Applies the callback to the item in this contact model with the
        given name (if such an item exists)."""
        if item_name in self.items:
            callback(self.items[item_name])

    def set_mesh_data(self, mesh_data: vtk.vtkPolyData, item_name: str,
                      add_callback, update_callback, view):
        """Implements VisualModel.set_contact_mesh_data."""
        if mesh_data is None:
            if item_name in self.items:
                om.removeFromObjectModel(self.items.pop(item_name).item)
        elif item_name in self.items:
            vis_item = self.items[item_name]
            update_callback(vis_item, mesh_data)
        else:
            suffix = self._contact_label_suffix(self._key)
            item = vis.PolyDataItem(item_name + suffix, mesh_data, view)
            om.addToObjectModel(item, self.folder)
            vis_item = VisualItem(item)
            self.items[item_name] = vis_item
            add_callback(vis_item, mesh_data)

    def set_debug_data(self, item_data: DebugData, item_name: str, view):
        """Implements VisualModel.set_contact_debug_data."""
        # The caller may instantiate an instance of DebugData (based on the
        # flag *requesting* visualization of some quantity), but find no
        # values to visualize. In that case, we don't want to add an item.
        # We detect it because the DebugData.append has no input
        # connections. This is part of the definition of "well defined"
        # data.
        if (item_data is None
                or item_data.append.GetNumberOfInputConnections(0) == 0):
            if item_name in self.items:
                om.removeFromObjectModel(self.items.pop(item_name).item)
        elif item_name in self.items:
            self.items[item_name].item.setPolyData(
                item_data.getPolyData())
            self.items[item_name].item.colorBy('RGB255')
        else:
            suffix = self._contact_label_suffix(self._key)
            item = vis.PolyDataItem(item_name + suffix,
                                    item_data.getPolyData(), view)
            om.addToObjectModel(item, self.folder)
            item.setProperty('Visible', True)
            item.setProperty('Alpha', 1.0)
            item.colorBy('RGB255')
            self.items[item_name] = VisualItem(item)


class _BodyContact:
    """Tracks the contact model components for each contact between a pair
    of bodies."""
    def __init__(self,
                 surface: lcmt_hydroelastic_contact_surface_for_viz,
                 root_folder: om.ContainerItem):
        """Constructs a BodyContact instance based on the data contained in
        the given `surface`. A folder (with a name based on the body names)
        will be place inside root_folder.

        Args:
            surface: The contact surface to add to the object model.
            root_folder: The parent directory for the contact folder."""
        self._folder = om.getOrCreateContainer(
            self._folder_name(surface), root_folder)

        # All of the contacts associated with this body pair. Each contact
        # is keyed by the geometry pair that generated it (see
        #  _contact_key()).
        self._contacts = {}

    def add_contact(self,
                    surface: lcmt_hydroelastic_contact_surface_for_viz,
                    timestamp: int):
        """Makes sure there's a Contact for the given surface in this
        instance (with a timestamp equal to the given value.

        Args:
            surface: The contact surface to add to the object model.
            timestamp: The timestamp at the time of creation."""
        key = _Contact.make_key(surface)
        if key in self._contacts:
            self._contacts[key].timestamp = timestamp
        else:
            self._contacts[key] = _Contact(surface, self._folder, timestamp)

    @staticmethod
    def make_key(surface: lcmt_hydroelastic_contact_surface_for_viz):
        """Given a `surface` returns a key reprepsenting the unique body pair
        in contact. We use it to organize the object model items and map it to
        the *folder* name in the object model.

        The folder contains the contact data between two bodies. A folder may
        contain data for multiple surfaces (when a body is represented with
        multiple geometries).

        Args:
            surface: The contact surface to add to the object model."""
        return (
            (surface.model1_name, surface.body1_name, surface.body1_unique),
            (surface.model2_name, surface.body2_name, surface.body2_unique))

    @staticmethod
    def _folder_name(surface: lcmt_hydroelastic_contact_surface_for_viz):
        """Given a contact surface message, constructs the corresponding folder
        name for the two bodies identified in `surface`'s data.

        Args:
            surface: The surface message."""
        name1 = _BodyContact.body_name(surface, 0)
        name2 = _BodyContact.body_name(surface, 1)
        return (f'({name1}, {name2}) Contact Data')

    def __len__(self):
        """Reports the number of contacts"""
        return len(self._contacts)

    def clean(self, timestamp: int):
        """Removes all contacts that don't match the given timestamp."""
        to_remove = []
        for key, contact in self._contacts.items():
            if contact.timestamp != timestamp:
                contact.clear()
                to_remove.append(key)
        for key in to_remove:
            del self._contacts[key]
        if len(self) == 0:
            self.clear()

    def clear(self):
        """Clears this body contact."""
        # Recursively remove all of the model items.
        om.removeFromObjectModel(self._folder)
        self._folder = None
        self._contacts = {}

    def set_debug_data(
      self, surface: lcmt_hydroelastic_contact_surface_for_viz,
      item_data: DebugData, item_name: str, view):
        """@pre add_contact() has been called on the given `surface`."""
        key = _Contact.make_key(surface)
        self._contacts[key].set_debug_data(item_data, item_name, view)

    def set_mesh_data(
      self, surface: lcmt_hydroelastic_contact_surface_for_viz,
      mesh_data: vtk.vtkPolyData, item_name, add_callback, update_callback,
      view):
        """@pre add_contact() has been called on the given `surface`."""
        key = _Contact.make_key(surface)
        self._contacts[key].set_mesh_data(mesh_data, item_name, add_callback,
                                          update_callback, view)

    def update_items(self, item_name: str, callback):
        for contact in self._contacts.values():
            contact.update_item(item_name, callback)

    @staticmethod
    def body_name(surface: lcmt_hydroelastic_contact_surface_for_viz,
                  body_index: int):
        """Generates the minimally unique name for a body identified by index
        in the given surface.

        Args:
            surface: The surface whose contact body is to be named.
            body_index: The index of the body to name (must be 0 or 1)."""
        key = _BodyContact.make_key(surface)
        model, body, is_unique = key[body_index]
        if not is_unique:
            return f'{model}/{body}'
        return body


class VisualModel:
    """Tracks all of the object model items used in the current model. It
    allows the visualizer to update existing components as messages come in
    rather than strictly rebuilding from scratch. This helps preserve GUI-level
    configurations (like setting something visible)."""
    def __init__(self, root_folder_name: str):
        """Constructs the visual model.
        VisualModel.set_view() must be called before any operations updating
        the view.

        Args:
            root_folder_name: The name of the object model folder that
                contains all hydroelastic visualization data.
        """
        self._root_folder = om.getOrCreateContainer(root_folder_name)
        self.view = None
        # All contacts between bodies are stored in this dictionary. The key
        # is the "body-pair key" (which later becomes a folder name for the
        # pair of contacting bodies).
        self._body_contacts = {}
        self._timestamp = 0

    def set_view(self, view):
        """Sets the view for the model.

        Args:
            view: The director view used to instantiate object model items."""
        self.view = view

    def clear(self):
        """Clears the entire model - all data for all contact surfaces are
        removed."""
        for body_contact in self._body_contacts.values():
            body_contact.clear()
        self._body_contacts = {}
        self._timestamp = 0

    def update_contact_directories(self,
                                   message: lcmt_contact_results_for_viz):
        """Given a hydroelastic contact message, updates the visual model's
        knowledge of what contacts are present. Contacts that are not present
        in the message get removed from the model. Contacts new in the message
        are added to the model.

        This does *not* populate any of the contact *items* for the contacts.
        """
        self._timestamp = self._timestamp + 1
        for surface in message.hydroelastic_contacts:
            body_pair_key = _BodyContact.make_key(surface)
            if body_pair_key not in self._body_contacts:
                body_contact = _BodyContact(surface, self._root_folder)
                self._body_contacts[body_pair_key] = body_contact
            # Add or update a contact for this body pair.
            self._body_contacts[body_pair_key].add_contact(surface,
                                                           self._timestamp)

        # Remove anything that didn't get persisted from the previous message
        # (detected by an expired timestamp).
        folders_to_remove = []
        for key, body_contact in self._body_contacts.items():
            body_contact.clean(self._timestamp)
            if len(body_contact) == 0:
                folders_to_remove.append(key)
        for key in folders_to_remove:
            # Python-ism, this also removes the entry from the dictionary.
            del self._body_contacts[key]

    def set_contact_debug_data(
      self, surface: lcmt_hydroelastic_contact_surface_for_viz,
      item_data: DebugData, item_name: str):
        """For the contact represented by the given `surface`, adds, updates,
        or removes (as appropriate) the named item with the given data.

        An item is added if the data is well defined and no item with the given
        name exists.

        An item is updated if the data is well defined and the named item
        already exists.

        An item is removed, if the data is *not* well defined and an item
        already exists.

        Args:
            surface: The contact surface to which this data belongs.
            item_data: The vtkAppendPolyData to add. It is "well defined"
                if it is *not* None and has had polygonal data added.
            item_name: The unique name associated with this data."""
        key = _BodyContact.make_key(surface)
        body_contact = self._body_contacts[key]
        body_contact.set_debug_data(surface, item_data, item_name, self.view)

    def set_contact_mesh_data(
      self, surface: lcmt_hydroelastic_contact_surface_for_viz,
      mesh_data: vtk.vtkPolyData, item_name: str, add_callback,
      update_callback):
        """For the contact represented by the given `surface`, adds, updates,
        or removes (as appropriate) the named item with the given mesh data.

        An item is added if the data is well defined and no item with the given
        name exists.

        An item is updated if the data is well defined and the named item
        already exists.

        An item is removed, if the data is *not* well defined and an item
        already exists.

        Args:
            surface: The contact surface to which this data belongs.
            mesh_data: The PolyDataItem to add. It is "well defined" if it
                is *not* None.
            item_name: The unique name associated with this data.
            add_callback: Functor of the form
                `f(item: VisualItem, mesh_data: vtk.vtkPolyData)` called when
                an item is newly added. The mesh_data is the data that has
                been assigned to the item. It can be used to provide custom
                configuration.
            update_callback: Functor of the form
                `f(item: VisualItem, mesh_data: vtk.vtkPolyData)` called when
                an item is updated. The mesh_data is the data that has been
                assigned to the item. It can be used to provide custom updating
                behavior."""
        key = _BodyContact.make_key(surface)
        body_contact = self._body_contacts[key]
        body_contact.set_mesh_data(surface, mesh_data, item_name, add_callback,
                                   update_callback, self.view)

    def update_items(self, item_name: str, callback):
        """Applies the callback to the named contact item in every contact."""
        for body_contact in self._body_contacts.values():
            body_contact.update_items(item_name, callback)


class HydroelasticContactVisualizer:
    def __init__(self):
        self._folder_name = 'Hydroelastic Contact'
        self._name = 'Hydroelastic Contact Visualizer'
        self._enabled = False
        # Subscriber to the CONTACT_RESULTS messages.
        self._contact_sub = None
        # Subscriber to the DRAKE_VIEWER_LOAD_ROBOT messages.
        self._load_sub = None

        self.set_enabled(True)

        # Visualization parameters
        # TODO(SeanCurtis-TRI): Find some way to persist these settings across
        #  invocations of drake visualizer. Config file, environment settings,
        #  something.
        self.color_map_mode = ColorMapModes.kFlameMap
        self.min_pressure = 0
        self.max_pressure = 1e8
        self.texture_size = 128
        self.show_edges = True
        self.edge_width = 2
        self.show_pressure = True
        self.max_pressure_observed = 0
        self.show_spatial_force = True
        self.show_traction_vectors = False
        self.show_slip_velocity_vectors = False
        self.magnitude_mode = ContactVisModes.kFixedLength
        self.global_scale = 0.3
        self.min_magnitude = 1e-4
        self.texture = create_texture(self.texture_size, FlameMap())
        # Persist the state so we can update without messages.
        self.visual_model = VisualModel(self._folder_name)
        self.message = None

        menu_bar = applogic.getMainWindow().menuBar()
        plugin_menu = get_sub_menu_or_make(menu_bar, '&Plugins')
        contact_menu = get_sub_menu_or_make(plugin_menu, '&Contacts')
        self.configure_action = contact_menu.addAction(
            'Configure &Hydroelastic Contact Visualization')
        self.configure_action.connect('triggered()', self.show_dialog)

        # Make the dialog a child of the main window so it closes when the app
        # closes.
        self.dlg = _ConfigDialog(self, applogic.getMainWindow())

        # Connect all of the widgets in the dialog to callbacks
        self.dlg.color_map_mode.currentIndexChanged.connect(self.set_color_map)
        self.dlg.min_pressure.editingFinished.connect(self.set_min_pressure)
        self.dlg.max_pressure.editingFinished.connect(self.set_max_pressure)
        self.dlg.show_pressure.toggled.connect(self.toggle_show_pressure)
        self.dlg.show_edges.toggled.connect(self.toggle_show_edges)
        self.dlg.edge_width.valueChanged.connect(self.set_edge_width)
        self.dlg.show_spatial_force.toggled.connect(
            self.toggle_show_spatial_force)
        self.dlg.show_traction_vectors.toggled.connect(
            self.toggle_show_traction_vectors)
        self.dlg.show_slip_velocity_vectors.toggled.connect(
            self.toggle_show_slip_velocity)
        self.dlg.magnitude_mode.currentIndexChanged.connect(
            self.set_magnitude_mode)
        self.dlg.global_scale.editingFinished.connect(self.set_global_scale)
        self.dlg.min_magnitude.editingFinished.connect(self.set_min_magnitude)
        self.dlg.reset_button.clicked.connect(self.clear_max_observed_pressure)

    def create_color_map(self):
        if self.color_map_mode == ColorMapModes.kFlameMap:
            return FlameMap()
        if self.color_map_mode == ColorMapModes.kIntensityMap:
            return IntensityMap()
        if self.color_map_mode == ColorMapModes.kTwoToneMap:
            return TwoToneMap()
        # Should never be here.
        assert False

    def show_dialog(self):
        self.dlg.show()

    def color_map_callback(self, vis_item, line_color):
        vis_item.item.actor.SetTexture(
            self.texture if self.show_pressure else None)
        vis_item.item.actor.GetProperty().SetEdgeColor(line_color)

    def set_color_map(self, new_index: int):
        """Slot for dialog widget"""
        if new_index != self.color_map_mode:
            self.color_map_mode = new_index
            color_map = self.create_color_map()
            self.texture = create_texture(self.texture_size, color_map)
            if self.visual_model:
                [r, g, b] = color_map.get_contrasting_color()
                line_color = [r*255, g*255, b*255]
                self.visual_model.update_items(
                    'Contact surface',
                    lambda vis_item: self.color_map_callback(vis_item,
                                                             line_color))
                applogic.getCurrentRenderView().render()

    def update_uv_transform(self, xform: vtk.vtkTransformTextureCoords):
        """Updates the uv transform to reflect the current pressure range
        settings.

        Args:
            xform: The transform to apply to the texture coordinates."""
        # Conceptually, we map the interval [min, max] -> [0, 1]. To achieve
        # this we simply translate and scale the pressure domain to the texture
        # coordinate domain. We use the vtkTransformTextureCoords to achieve
        # this with some surprising implementation details. The scale factor is
        # as one would expect but the translation is not.
        #
        #   - The scale is simply 1 / (max - min). We introduce a small epsilon
        #     in the denominator to prevent division by zero.
        #   - The scaling is applied *before* the translation. That means we
        #     don't offset the data by min, but by min * scale.
        #   - We achieve translation by setting the *origin*. To translate a
        #     target d units, we must put the origin at -d.
        #
        # Finally, We rely on the texture map *clamping* its values for
        # pressure values less than min or greater than max.
        pressure_scale = 1.0 / (self.max_pressure - self.min_pressure + 1e-10)
        xform.SetOrigin(-self.min_pressure * pressure_scale, 0, 0)
        xform.SetScale(pressure_scale, 1, 1)

    def update_pressure_range(self):
        """Updates all pressure patches to reflect the current pressure range
        settings."""
        def update_item(vis_item):
            # We assume that this is only called on items with user data where
            # the first item is the texture transform.
            xform = vis_item.user_data[0]
            self.update_uv_transform(xform)

        if self.visual_model:
            self.visual_model.update_items('Contact surface', update_item)
            applogic.getCurrentRenderView().render()

    def set_min_pressure(self):
        """Slot for dialog widget"""
        new_value = float(self.dlg.min_pressure.text)
        if new_value != self.min_pressure:
            self.min_pressure = new_value
            self.update_pressure_range()
            self.update_visual_data_from_message()

    def set_max_pressure(self):
        """Slot for dialog widget"""
        new_value = float(self.dlg.max_pressure.text)
        if new_value != self.max_pressure:
            self.max_pressure = new_value
            self.update_pressure_range()
            self.update_visual_data_from_message()

    def edges_visible(self):
        return self.show_edges and self.edge_width > 0

    def update_mesh_drawing_callback(self, vis_item):
        """Callback for coordinating changes to how the contact surface is
        drawn."""
        vis_item.item.actor.GetProperty().SetLineWidth(self.edge_width)
        vis_item.item.actor.SetTexture(
            self.texture if self.show_pressure else None)
        visible = self.show_pressure or self.edges_visible()
        # TODO(SeanCurtis-TRI): Perhaps it would be better if I popped the item
        #  out of the object model
        vis_item.item.setProperty('Visible', visible)
        if visible:
            # Only change the drawing mode if it's visible.
            if self.show_pressure:
                mode = 'Surface'
                if self.edges_visible():
                    mode += ' with edges'
            else:
                mode = 'Wireframe'
            vis_item.item.setProperty('Surface Mode', mode)

    def toggle_show_pressure(self, state):
        """Slot for dialog widget"""
        if self.show_pressure != state:
            self.show_pressure = state
            if self.visual_model:
                self.visual_model.update_items(
                    'Contact surface', self.update_mesh_drawing_callback)

    def toggle_show_edges(self, state):
        """Slot for dialog widget"""
        if self.show_edges != state:
            self.show_edges = state
            if self.visual_model:
                self.visual_model.update_items(
                    'Contact surface', self.update_mesh_drawing_callback)

    def set_edge_width(self, width):
        """Slot for dialog widget."""
        if self.edge_width != width:
            self.edge_width = width
            if self.visual_model:
                self.visual_model.update_items(
                    'Contact surface', self.update_mesh_drawing_callback)

    def toggle_show_spatial_force(self, state):
        """Slot for dialog widget"""
        self.show_spatial_force = state
        self.update_visual_data_from_message()

    def toggle_show_traction_vectors(self, state):
        """Slot for dialog widget"""
        self.show_traction_vectors = state
        self.update_visual_data_from_message()

    def toggle_show_slip_velocity(self, state):
        """Slot for dialog widget"""
        self.show_slip_velocity_vectors = state
        self.update_visual_data_from_message()

    def set_magnitude_mode(self, new_index):
        """Slot for dialog widget"""
        if new_index != self.magnitude_mode:
            self.magnitude_mode = new_index
            self.update_visual_data_from_message()

    def set_global_scale(self):
        """Slot for dialog widget"""
        new_value = float(self.dlg.global_scale.text)
        if new_value != self.global_scale:
            self.global_scale = new_value
            self.update_visual_data_from_message()

    def set_min_magnitude(self):
        """Slot for dialog widget"""
        new_value = float(self.dlg.min_magnitude.text)
        if new_value != self.min_magnitude:
            self.min_magnitude = new_value
            self.update_visual_data_from_message()

    def clear_max_observed_pressure(self):
        """Slot for dialog widget"""
        self.max_pressure_observed = -1
        # Also causes the dialog to update.
        self.update_max_pressure(0)

    def update_max_pressure(self, pressure):
        """Tests to see if the maximum pressure needs to be increased. If so,
        updates the dialog."""
        if pressure > self.max_pressure_observed:
            self.max_pressure_observed = pressure
            # Note: This is a *horrible* hack rendered necessary because
            # PythonQt doesn't provide QtCore.QObject *or* QtCore.pyqtSignal so
            # we can't define signals on this class that connect to dialog
            # slots.
            self.dlg.set_max_pressure(pressure)

    def add_subscriber(self):
        if self._contact_sub is not None:
            return

        self._contact_sub = lcmUtils.addSubscriber(
            'CONTACT_RESULTS',
            messageClass=lcmt_contact_results_for_viz,
            callback=self.handle_message)
        print(self._name + ' subscriber added.')
        self._load_sub = lcmUtils.addSubscriber(
            'DRAKE_VIEWER_LOAD_ROBOT',
            messageClass=lcmt_viewer_load_robot,
            callback=self.clear_on_load)

    def remove_subscriber(self):
        if self._contact_sub is None:
            return

        lcmUtils.removeSubscriber(self._contact_sub)
        self._contact_sub = None
        om.removeFromObjectModel(om.findObjectByName(self._folder_name))
        print(self._name + ' subscriber removed.')

        lcmUtils.removeSubscriber(self._load_sub)
        self._load_sub = None

    def is_enabled(self):
        return self._enabled

    def set_enabled(self, enable):
        self._enabled = enable
        if enable:
            self.add_subscriber()
        else:
            self.remove_subscriber()

    def clear_on_load(self, msg):
        """Clears the entire visual model because it detected a robot load
        message."""
        if self.visual_model is not None:
            self.visual_model.clear()
            self.message = None

    def handle_message(self, msg):
        # Limits the rate of message handling, since redrawing is done in the
        # message handler.
        self._contact_sub.setSpeedLimit(30)

        # Always set the active view, just to be safe.
        self.visual_model.set_view(applogic.getCurrentRenderView())
        self.visual_model.update_contact_directories(msg)
        self.message = msg
        self.update_visual_data_from_message()

    def make_mesh(self, surface: lcmt_hydroelastic_contact_surface_for_viz):
        """
        Creates a vtk.vtkPolyData instance from the given contact surface
        message.

        Args:
            surface: The contact surface to add to the object model."""
        p_WVs = np.empty((surface.num_vertices, 3))
        uvs = np.empty((surface.num_vertices, 2))
        for i in range(surface.num_vertices):
            p_WV = surface.p_WV[i]
            p_WVs[i, :] = (p_WV.x, p_WV.y, p_WV.z)
            uvs[i, :] = (surface.pressure[i], 0)
            self.update_max_pressure(surface.pressure[i])

        vtk_polys = vtk.vtkCellArray()
        i = 0
        poly_count = 0
        while i < surface.poly_data_int_count:
            count = surface.poly_data[i]
            poly_count += 1
            i += count + 1
        vtk_polys.Allocate(poly_count)
        i = 0
        while i < surface.poly_data_int_count:
            vertex_count = surface.poly_data[i]
            i += 1
            poly_indices = surface.poly_data[i:i + vertex_count]
            vtk_polys.InsertNextCell(vertex_count, poly_indices)
            i += vertex_count
        vtk_polydata = vtk.vtkPolyData()
        vtk_polydata.SetPoints(vnp.getVtkPointsFromNumpy(p_WVs))
        vtk_polydata.SetPolys(vtk_polys)
        vtk_polydata.GetPointData().SetTCoords(
            vnp.getVtkFromNumpy(uvs))
        return vtk_polydata

    def update_visual_data_from_message(self):
        """Updates the visual state based on the currently owned message. This
        reconstructs *everything* as if the message were newly received."""
        msg = self.message

        if msg is None:
            return

        # TODO(SeanCurtis-TRI): This is used to update *current* data based on
        #  configuration changes. *Any* change triggers a re-evaluation of
        #  *all* visualization data. It's a simple, yet *huge* hammer.
        #  Investigate making the updating more targeted in the future.

        # The scale value attributable to auto-scale.
        auto_force_scale = 1.0
        auto_moment_scale = 1.0
        auto_traction_scale = 1.0
        auto_slip_velocity_scale = 1.0
        max_force = -1
        max_moment = -1
        max_traction = -1
        max_slip_speed = -1

        # Determine scaling magnitudes if autoscaling is activated.
        if self.magnitude_mode == ContactVisModes.kAutoScale:
            for surface in msg.hydroelastic_contacts:
                if self.show_spatial_force:
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

            # Compute scaling factors. We don't want division by zero.
            # We don't want division by negative numbers.
            if max_force > 0:
                auto_force_scale = 1.0 / max_force
            if max_moment > 0:
                auto_moment_scale = 1.0 / max_moment
            if max_traction > 0:
                auto_traction_scale = 1.0 / max_traction
            if max_slip_speed > 0:
                auto_slip_velocity_scale = 1.0 / max_slip_speed

        # TODO(drum) Consider exiting early if no visualization options are
        # enabled.
        for surface in msg.hydroelastic_contacts:
            # For each quantity (spatial force, pressure field, etc.), we want
            # to update the visual model explicitly by saying whether it is
            # present or not. So, we initialize the data to None in all cases
            # and *always* update self.visual_model. If the data is None, the
            # quantity is explicitly off. Otherwise, it'll be on given the
            # data generated in the block.

            # Draw the spatial force.
            force_data = None
            if self.show_spatial_force:
                force_data = DebugData()
                point = np.array([surface.centroid_W[0],
                                  surface.centroid_W[1],
                                  surface.centroid_W[2]])
                # This is actually the force on body 1. It is documented as
                # such in the lcm message.
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

                    force_data.addArrow(
                        start=point,
                        end=point + auto_force_scale * force * scale,
                        tubeRadius=0.002,
                        headRadius=0.004, color=[1, 0, 0])

                # Draw the moment arrow if it's of sufficient magnitude.
                if moment_mag > self.min_magnitude:
                    scale = self.global_scale
                    if self.magnitude_mode == ContactVisModes.kFixedLength:
                        # magnitude must be > 0 otherwise this moment would be
                        # skipped.
                        scale /= moment_mag

                    force_data.addArrow(
                        start=point,
                        end=point + auto_moment_scale * moment * scale,
                        tubeRadius=0.002,
                        headRadius=0.004, color=[0, 0, 1])
            # TODO(SeanCurtis-TRI) See show_point_pair_contact.py. But if we
            #  ever had a single body represented with multiple contact
            #  geometries, we could end up with body pairs (A, B) and (B, A).
            #  This is less likely with hydro than with point pair contact, so
            #  resolving this is less urgent.

            # The force is documented as acting on body 1 (out of bodies 1 and
            # 2) -- so we use index 0.
            body_name = _BodyContact.body_name(surface, 0)
            self.visual_model.set_contact_debug_data(
                surface, force_data, f"Spatial force on {body_name}")

            # Iterate over all quadrature points, drawing traction and slip
            # velocity vectors.
            traction_data = None
            slip_data = None
            if self.show_traction_vectors or self.show_slip_velocity_vectors:
                traction_data = DebugData()
                slip_data = DebugData()

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
                            traction_data.addArrow(
                                start=origin, end=origin + offset,
                                tubeRadius=0.00125,
                                headRadius=0.0025, color=[1, 0, 1])
                        else:
                            traction_data.addSphere(
                                center=origin,
                                radius=0.00125,
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
                            slip_data.addArrow(
                                start=origin, end=origin + offset,
                                tubeRadius=0.00125,
                                headRadius=0.0025, color=[0, 1, 1])
                        else:
                            slip_data.addSphere(
                                center=origin,
                                radius=0.00125,
                                color=[0, 1, 1])
            self.visual_model.set_contact_debug_data(
                surface, traction_data, "Traction")
            self.visual_model.set_contact_debug_data(
                surface, slip_data, "Slip velocity")

            vtk_polydata = None
            if self.show_pressure or self.edges_visible():
                vtk_polydata = self.make_mesh(surface)

            self.visual_model.set_contact_mesh_data(
                surface, vtk_polydata, 'Contact surface',
                self.add_pressure_mesh_cb, self.update_pressure_mesh_cb)

    def add_pressure_mesh_cb(self, vis_item: VisualItem,
                             mesh_data: vtk.vtkPolyData):
        """The callback supplied to the VisualModel for when adding a pressure
        mesh."""
        # We subvert the standard pipeline that director uses to inject
        # texture coordinate transformation (moving and scaling). Scaling the
        # texture coordinates provides a superior result. It *seems* that
        # it causes the mapping to happen per-fragment instead of per-vertex.
        item = vis_item.item
        xform = vtk.vtkTransformTextureCoords()
        xform.SetInputData(mesh_data)
        self.update_uv_transform(xform)
        # Stash the xform to be used in update.
        vis_item.user_data = (xform, )

        mapper = item.actor.GetMapper()
        mapper.SetInputData(None)
        mapper.SetInputConnection(xform.GetOutputPort())
        if self.show_pressure:
            item.actor.SetTexture(self.texture)
        else:
            item.actor.SetTexture(None)

        # Now configure hidden-line drawing.
        # Disable lighting so the surface appears the same looking from either
        # side.
        item.actor.GetProperty().SetLighting(False)
        if self.edge_width > 0:
            color_map = self.create_color_map()
            [r, g, b] = color_map.get_contrasting_color()
            line_color = [r*255, g*255, b*255]
            item.actor.GetProperty().SetEdgeColor(line_color)
            item.actor.GetProperty().SetLineWidth(self.edge_width)
            if self.show_pressure:
                item.setProperty('Surface Mode', 'Surface with edges')
                item.actor.SetTexture(self.texture)
            else:
                item.setProperty('Surface Mode', 'Wireframe')
                item.actor.SetTexture(None)
        else:
            item.setProperty('Surface Mode', 'Surface')

    def update_pressure_mesh_cb(self, vis_item: VisualItem,
                                mesh_data: vtk.vtkPolyData):
        """The callback supplied to the VisualModel for when updating a
        pressure mesh."""
        # In updating the pressure, we can't rely on director to
        # connect the mesh data in correctly, we've stashed the texture
        # coord transform into our user data; we'll connect into that.
        vis_item.item.polyData = mesh_data
        vis_item.user_data[0].SetInputData(mesh_data)


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
