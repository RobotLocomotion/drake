import copy
import logging
import numpy as np
import sys
import time

from drake import (
    lcmt_contact_results_for_viz,
    lcmt_point_cloud,
    lcmt_point_cloud_field,
    lcmt_viewer_draw,
    lcmt_viewer_geometry_data,
    lcmt_viewer_link_data,
    lcmt_viewer_load_robot,
)
from pydrake.common import (
    configure_logging,
)
from pydrake.common.eigen_geometry import (
    Quaternion,
)
from pydrake.geometry import (
    Box,
    Capsule,
    Cylinder,
    Ellipsoid,
    Mesh,
    Meshcat,
    MeshcatParams,
    Rgba,
    Sphere,
)
from pydrake.lcm import (
    DrakeLcm,
)
from pydrake.math import (
    RigidTransform,
    RotationMatrix,
)
from pydrake.multibody.meshcat import (
    _HydroelasticContactVisualizer,
    _HydroelasticContactVisualizerItem,
    _PointContactVisualizer,
    _PointContactVisualizerItem,
    ContactVisualizerParams,
)
from pydrake.perception import (
    BaseField,
    Fields,
    PointCloud,
)

_logger = logging.getLogger("drake")


class _Slider:
    """A slider with range [small-positive-value to 1.0]."""

    def __init__(self, meshcat, name):
        self._meshcat = meshcat
        self._name = name
        self._value = 1.0
        self._exists = False

    def realize(self):
        """Ensures the slider is created and visible."""
        # Avoid a redundant slider crash if the user runs a sending program
        # twice with the same meldis still running.
        if not self._exists:
            self._exists = True
            self._meshcat.AddSlider(self._name, 0.02, 1.0, 0.02, self._value)

    def read(self):
        """Returns a pair of the current value, and a flag that is True if the
        value has changed since the last read().

        Clients *must* call realize() before calling read().
        """
        assert self._exists, "Call realize() before calling read()"
        value = self._meshcat.GetSliderValue(self._name)
        value_changed = (value != self._value)
        self._value = value
        return value, value_changed


class _ViewerApplet:
    """Displays lcmt_viewer_load_robot and lcmt_viewer_draw into MeshCat."""

    def __init__(self, *, meshcat, path, alpha_slider_name,
                 should_accept_link=None, start_visible=True):
        """Constructs an applet.

        If should_accept_link is given, only links where
        `should_accept_link(link.name)` is true will be displayed. (N.B. This
        predicate only applies to rigid bodies; it does not apply to deformable
        geometries, which do not have a concept of a "link".)
        """
        self._meshcat = meshcat
        self._path = path
        self._load_message = None
        self._alpha_slider = _Slider(meshcat, alpha_slider_name)
        if should_accept_link is not None:
            self._should_accept_link = should_accept_link
        else:
            self._should_accept_link = lambda _: True
        self._start_visible = start_visible
        self._geom_paths = []
        self._geom_colors = []

        # Initialize ourself with an empty load message.
        self.on_viewer_load(message=lcmt_viewer_load_robot())

    def on_viewer_load(self, message):
        """Handler for lcmt_viewer_load."""
        # Ignore duplicate load messages. This is important for visualization
        # performance when the user is repeatedly viewing the same simulation
        # over and over again, since reloading a scene into Meshcat has high
        # latency.
        if self._load_message is not None:
            if (message.num_links == self._load_message.num_links
                    and message.encode() == self._load_message.encode()):
                _logger.info("Ignoring duplicate load message")
                return

        # The semantics of a load message is to reset the entire scene.
        self._meshcat.Delete(path=self._path)

        self._waiting_for_first_draw_message = True
        self._load_message = message

    def _build_links(self):
        # Make all of our (ViewerApplet's) geometry invisible so that the
        # lcmt_viewer_load geometry does not clutter up the scene until we
        # are given its poses in a lcmt_viewer_draw message.
        self._set_visible(False)

        message = self._load_message

        # Add the links and their geometries.
        self._geom_paths = []
        self._geom_colors = []
        for link in message.link:
            if not self._should_accept_link(link.name):
                continue
            robot_num = link.robot_num
            link_name = link.name.replace("::", "/")
            link_path = f"{self._path}/{robot_num}/{link_name}"
            for j, geom in enumerate(link.geom):
                geom_path = f"{link_path}/{j}"
                shape, rgba, pose = self._convert_geom(geom)
                if shape is None:
                    continue
                self._geom_paths.append(geom_path)
                self._geom_colors.append(rgba)
                self._meshcat.SetObject(path=geom_path, shape=shape, rgba=rgba)
                self._meshcat.SetTransform(path=geom_path, X_ParentPath=pose)
        assert len(self._geom_paths) == len(self._geom_colors)

    def on_viewer_draw(self, message):
        """Handler for lcmt_viewer_draw."""
        for i in range(message.num_links):
            if not self._should_accept_link(message.link_name[i]):
                continue
            link_name = message.link_name[i].replace("::", "/")
            robot_num = message.robot_num[i]
            link_path = f"{self._path}/{robot_num}/{link_name}"
            pose = self._to_pose(message.position[i], message.quaternion[i])
            self._meshcat.SetTransform(path=link_path, X_ParentPath=pose)
        if self._waiting_for_first_draw_message:
            self._waiting_for_first_draw_message = False
            self._build_links()
            self._alpha_slider.realize()
            if self._start_visible:
                self._set_visible(True)
            self.on_poll(force=True)

    def on_poll(self, force=False):
        if self._waiting_for_first_draw_message:
            return
        value, value_changed = self._alpha_slider.read()
        if force or value_changed:
            max_alpha = max([x.a() for x in self._geom_colors], default=1.0)
            for k in range(len(self._geom_paths)):
                path = self._geom_paths[k]
                new_color = copy.deepcopy(self._geom_colors[k])
                if max_alpha == 0:
                    new_color.update(a=value)
                else:
                    new_color.update(a=value * new_color.a())
                self._meshcat.SetProperty(path, "color", new_color.rgba)

    def on_viewer_draw_deformable(self, message):
        """Handler for lcmt_viewer_link_data."""
        link_name = message.name
        robot = message.robot_num
        link_path = f"{self._path}/{robot}/{link_name}"
        for i, geom in enumerate(message.geom):
            geom_name = geom.string_data
            geom_path = f"{link_path}/{geom_name}"
            vertices, faces, rgba, pose = self._convert_deformable_geom(geom)
            self._meshcat.SetTriangleMesh(
                path=geom_path, vertices=vertices, faces=faces, rgba=rgba)
            self._meshcat.SetTransform(path=link_path, X_ParentPath=pose)
        if self._waiting_for_first_draw_message:
            self._waiting_for_first_draw_message = False
            self._set_visible(True)

    def _set_visible(self, value):
        self._meshcat.SetProperty(self._path, property="visible", value=value)

    def _convert_deformable_geom(self, geom):
        """Given an lcmt_viewer_geometry_data, parses it into a tuple of
        (vertices, faces, Rgba, RigidTransform) if the geometry type is a
        MESH.
        """
        assert geom.type == lcmt_viewer_geometry_data.MESH
        num_verts = int(geom.float_data[0])
        num_faces = int(geom.float_data[1])
        # The first two floats encode the number of vertices and number of
        # triangles.
        v_start_index = 2
        f_start_index = v_start_index + 3 * num_verts
        vertices = np.array(geom.float_data[v_start_index:f_start_index])
        faces = np.array(geom.float_data[f_start_index:]).astype(int)
        vertices = np.reshape(vertices, (3, num_verts), order='F')
        faces = np.reshape(faces, (3, num_faces), order='F')
        rgba = Rgba(*geom.color)
        pose = self._to_pose(geom.position, geom.quaternion)
        return (vertices, faces, rgba, pose)

    def _convert_geom(self, geom):
        """Given an lcmt_viewer_geometry_data, parses it into a tuple of
        (Shape, Rgba, RigidTransform).
        """
        shape = None
        if geom.type == lcmt_viewer_geometry_data.BOX:
            (width, depth, height) = geom.float_data
            shape = Box(width=width, depth=depth, height=height)
        elif geom.type == lcmt_viewer_geometry_data.CAPSULE:
            (radius, length) = geom.float_data
            shape = Capsule(radius=radius, length=length)
        elif geom.type == lcmt_viewer_geometry_data.CYLINDER:
            (radius, length) = geom.float_data
            shape = Cylinder(radius=radius, length=length)
        elif geom.type == lcmt_viewer_geometry_data.ELLIPSOID:
            (a, b, c) = geom.float_data
            shape = Ellipsoid(a=a, b=b, c=c)
        elif geom.type == lcmt_viewer_geometry_data.MESH and geom.string_data:
            # A mesh to be loaded from a file.
            (scale_x, scale_y, scale_z) = geom.float_data
            filename = geom.string_data
            assert scale_x == scale_y and scale_y == scale_z
            shape = Mesh(filename=filename, scale=scale_x)
        elif geom.type == lcmt_viewer_geometry_data.MESH:
            assert not geom.string_data
            # A mesh with the data inline, i.e.,
            #   V | T | v0 | v1 | ... vN | t0 | t1 | ... | tM
            # where
            #   V: The number of vertices.
            #   T: The number of triangles.
            #   N: 3V, the number of floating point values for the V vertices.
            #   M: 3T, the number of vertex indices for the T triangles.
            _logger.warning("Meldis cannot yet display hydroelastic collision "
                            "meshes; that geometry will be ignored.")
        elif geom.type == lcmt_viewer_geometry_data.SPHERE:
            (radius,) = geom.float_data
            shape = Sphere(radius=radius)
        else:
            _logger.warning(f"Unknown geom.type of {geom.type}")
            return (None, None, None)
        rgba = Rgba(*geom.color)
        pose = self._to_pose(geom.position, geom.quaternion)
        return (shape, rgba, pose)

    @staticmethod
    def _to_pose(position, quaternion):
        """Given pose parts of an lcmt_viewer_geometry_data, parses it into a
        RigidTransform.
        """
        (p_x, p_y, p_z) = position
        (q_w, q_x, q_y, q_z) = quaternion
        return RigidTransform(
            R=RotationMatrix(
                quaternion=Quaternion(
                    w=q_w,
                    x=q_x,
                    y=q_y,
                    z=q_z,
                ),
            ),
            p=(p_x, p_y, p_z))


class _ContactApplet:
    """Displays lcmt_contact_results_for_viz into Meshcat."""

    def __init__(self, *, meshcat):
        # By default, don't show any contact illustrations.
        meshcat.SetProperty("/CONTACT_RESULTS", "visible", True)

        # Add point visualization.
        params = ContactVisualizerParams()
        params.prefix = "/CONTACT_RESULTS/point"
        self._point_helper = _PointContactVisualizer(meshcat, params)

        # Add hydroelastic visualization.
        params = ContactVisualizerParams()
        params.prefix = "/CONTACT_RESULTS/hydroelastic"
        self._hydro_helper = _HydroelasticContactVisualizer(meshcat, params)

    # Converts poly_data from a hydro lcm message to numpy array.
    def convert_faces(self, poly_data):
        poly_index = 0
        faces = []
        while poly_index < len(poly_data):
            poly_i_num_vertices = poly_data[poly_index]
            vertex_0_index = poly_index + 1
            vertex_0_global_index = poly_data[vertex_0_index]
            for i in range(1, poly_i_num_vertices - 1):
                vertex_1_global_index = poly_data[vertex_0_index + i]
                vertex_2_global_index = poly_data[vertex_0_index + i + 1]
                faces.append([vertex_0_global_index,
                              vertex_1_global_index,
                              vertex_2_global_index])
            poly_index += poly_i_num_vertices + 1
        return np.array(faces).transpose()

    # Converts verts from hydro lcm message to numpy array
    def convert_verts(self, p_WV):
        verts = np.empty((3, len(p_WV)))
        for i in range(len(p_WV)):
            verts[0, i] = p_WV[i].x
            verts[1, i] = p_WV[i].y
            verts[2, i] = p_WV[i].z
        return verts

    def get_full_names(self, item):
        name1 = []
        name2 = []

        # Use model instance name if necessary.
        if not item.body1_unique:
            name1.append(item.model1_name)

        if not item.body2_unique:
            name2.append(item.model2_name)

        name1.append(item.body1_name)
        name2.append(item.body2_name)

        # Use geometry name if necessary.
        if item.collision_count1 > 1:
            name1.append(item.geometry1_name)

        if item.collision_count2 > 1:
            name2.append(item.geometry2_name)

        return (".".join(name1), ".".join(name2))

    def on_contact_results(self, message):
        """Handler for lcmt_contact_results_for_viz. Note that only point
           hydroelastic contact force and moment vectors are shown; contact
           surface and pressure are not shown.
        """

        # Handle point contact pairs
        viz_items = []
        for lcm_item in message.point_pair_contact_info:
            viz_items.append(_PointContactVisualizerItem(
                body_A=lcm_item.body1_name,
                body_B=lcm_item.body2_name,
                contact_force=lcm_item.contact_force,
                contact_point=lcm_item.contact_point))
        self._point_helper.Update(0, viz_items)

        # Handle hydroelastic contact pairs
        viz_items = []
        for lcm_item in message.hydroelastic_contacts:
            (name1, name2) = self.get_full_names(lcm_item)
            viz_items.append(_HydroelasticContactVisualizerItem(
                body_A=name1,
                body_B=name2,
                centroid_W=lcm_item.centroid_W,
                force_C_W=lcm_item.force_C_W,
                moment_C_W=lcm_item.moment_C_W,
                p_WV=self.convert_verts(lcm_item.p_WV),
                faces=self.convert_faces(lcm_item.poly_data),
                pressure=lcm_item.pressure))
        self._hydro_helper.Update(0, viz_items)


class _PointCloudApplet:
    """Displays lcmt_point_cloud into MeshCat."""

    _POINT_CLOUD_FIELDS = (
        # (name, byte_offset, datatype, count)
        ("x", 0, lcmt_point_cloud_field.FLOAT32, 1),
        ("y", 4, lcmt_point_cloud_field.FLOAT32, 1),
        ("z", 8, lcmt_point_cloud_field.FLOAT32, 1),
        ("rgb", 12, lcmt_point_cloud_field.UINT32, 1),
        ("normal_x", 16, lcmt_point_cloud_field.FLOAT32, 1),
        ("normal_y", 20, lcmt_point_cloud_field.FLOAT32, 1),
        ("normal_z", 24, lcmt_point_cloud_field.FLOAT32, 1),
    )
    """The supported fields and their data types of a point cloud. An XYZ,
    XYZRGB, and XYZRGBNormal cloud will have the first three, first four, and
    all the fields in this particular order.
    """

    def __init__(self, *, meshcat):
        self._meshcat = meshcat
        self._already_warned_channel_names = set()

    def _validate_and_get_fields(self, message):
        """Checks the point cloud LCM message and returns the corresponding
        `Fields` for the PointCloud object. Either XYZ, XYZRGB, or XYZRGBNormal
        cloud with the exact data type is supported.
        """
        if message.flags != lcmt_point_cloud.IS_STRICTLY_FINITE:
            return None

        if message.num_fields not in (3, 4, 7):
            return None

        for i in range(message.num_fields):
            (name, byte_offset, datatype, count) = self._POINT_CLOUD_FIELDS[i]
            if (
                message.fields[i].name != name
                or message.fields[i].byte_offset != byte_offset
                or message.fields[i].datatype != datatype
                or message.fields[i].count != count
            ):
                return None

        if message.num_fields == 3:
            return Fields(BaseField.kXYZs)
        elif message.num_fields == 4:
            return Fields(BaseField.kXYZs | BaseField.kRGBs)
        else:
            return Fields(
                BaseField.kXYZs | BaseField.kRGBs | BaseField.kNormals
            )

    def _channel_to_meshcat_path(self, channel):
        assert channel.startswith("DRAKE_POINT_CLOUD")
        if channel == "DRAKE_POINT_CLOUD":
            return "/POINT_CLOUD/default"
        else:
            # E.g., `DRAKE_POINT_CLOUD_FOO` => `/POINT_CLOUD/FOO`.
            suffix = channel[len("DRAKE_POINT_CLOUD_"):]
            return f"/POINT_CLOUD/{suffix}"

    def on_point_cloud(self, channel, message):
        """Handler for lcmt_point_cloud.

        Validates and converts the lcmt_point_cloud message to a PointCloud
        object for display.
        """
        cloud_fields = self._validate_and_get_fields(message)
        if cloud_fields is None:
            # Throttle warning messages to one per channel.
            if channel not in self._already_warned_channel_names:
                self._already_warned_channel_names.add(channel)
                _logger.warn(f"Unsupported point cloud data from {channel}.")
            return

        # Transform the raw data into an N x num_fields array.
        raw_data = np.frombuffer(message.data, dtype=np.float32).reshape(
            -1, message.num_fields
        )
        num_points = raw_data.shape[0]

        cloud = PointCloud(num_points, cloud_fields)
        xyzs = raw_data[:, 0:3]
        cloud.mutable_xyzs()[:] = xyzs.transpose()
        if message.num_fields > 3:
            rgbs_with_padding = (
                raw_data[:, 3].astype(np.float32).view(np.uint8).reshape(-1, 4)
            )
            rgbs = rgbs_with_padding[:, 0:3]
            cloud.mutable_rgbs()[:] = rgbs.transpose()
        if message.num_fields > 4:
            normals = raw_data[:, 4:]
            cloud.mutable_normals()[:] = normals.transpose()

        self._meshcat.SetObject(
            path=self._channel_to_meshcat_path(channel),
            cloud=cloud,
            point_size=0.01
        )


class Meldis:
    """
    MeshCat LCM Display Server (MeLDiS)

    Offers a MeshCat visualization server that listens for and draws Drake's
    legacy LCM visualization messages.

    If the meshcat_host parameter is not supplied, 'localhost' will be used by
    default.

    Refer to the pydrake.visualization.meldis module docs for details.
    """

    def __init__(self, *, meshcat_host=None, meshcat_port=None):
        # Bookkeeping for update throttling.
        self._last_update_time = time.time()

        # Bookkeeping for subscriptions, keyed by LCM channel name.
        self._message_types = {}
        self._message_handlers = {}
        self._message_pending_data = {}

        self._poll_handlers = []

        self._lcm = DrakeLcm()
        lcm_url = self._lcm.get_lcm_url()
        _logger.info(f"Meldis is listening for LCM messages at {lcm_url}")

        params = MeshcatParams(host=meshcat_host or "localhost",
                               port=meshcat_port,
                               show_stats_plot=False)
        self.meshcat = Meshcat(params=params)

        def is_inertia_link(link_name):
            return "::InertiaVisualizer::" in link_name

        def is_not_inertia_link(link_name):
            return not is_inertia_link(link_name)

        default_viewer = _ViewerApplet(meshcat=self.meshcat,
                                       path="/DRAKE_VIEWER",
                                       alpha_slider_name="Viewer α",
                                       should_accept_link=is_not_inertia_link)
        self._subscribe(channel="DRAKE_VIEWER_LOAD_ROBOT",
                        message_type=lcmt_viewer_load_robot,
                        handler=default_viewer.on_viewer_load)
        self._subscribe(channel="DRAKE_VIEWER_DRAW",
                        message_type=lcmt_viewer_draw,
                        handler=default_viewer.on_viewer_draw)
        self._subscribe(channel="DRAKE_VIEWER_DEFORMABLE",
                        message_type=lcmt_viewer_link_data,
                        handler=default_viewer.on_viewer_draw_deformable)
        self._poll(handler=default_viewer.on_poll)

        inertia_viewer = _ViewerApplet(meshcat=self.meshcat,
                                       path="/Inertia Visualizer",
                                       alpha_slider_name="Inertia α",
                                       should_accept_link=is_inertia_link,
                                       start_visible=False)
        inertia_viewer._alpha_slider._value = 0.5
        self._subscribe(channel="DRAKE_VIEWER_LOAD_ROBOT",
                        message_type=lcmt_viewer_load_robot,
                        handler=inertia_viewer.on_viewer_load)
        self._subscribe(channel="DRAKE_VIEWER_DRAW",
                        message_type=lcmt_viewer_draw,
                        handler=inertia_viewer.on_viewer_draw)
        self._poll(handler=inertia_viewer.on_poll)

        illustration_viewer = _ViewerApplet(meshcat=self.meshcat,
                                            path="/Visual Geometry",
                                            alpha_slider_name="Visual α")
        self._subscribe(channel="DRAKE_VIEWER_LOAD_ROBOT_ILLUSTRATION",
                        message_type=lcmt_viewer_load_robot,
                        handler=illustration_viewer.on_viewer_load)
        self._subscribe(channel="DRAKE_VIEWER_DRAW_ILLUSTRATION",
                        message_type=lcmt_viewer_draw,
                        handler=illustration_viewer.on_viewer_draw)
        self._poll(handler=illustration_viewer.on_poll)

        proximity_viewer = _ViewerApplet(meshcat=self.meshcat,
                                         path="/Collision Geometry",
                                         alpha_slider_name="Collision α",
                                         start_visible=False)
        self._subscribe(channel="DRAKE_VIEWER_LOAD_ROBOT_PROXIMITY",
                        message_type=lcmt_viewer_load_robot,
                        handler=proximity_viewer.on_viewer_load)
        self._subscribe(channel="DRAKE_VIEWER_DRAW_PROXIMITY",
                        message_type=lcmt_viewer_draw,
                        handler=proximity_viewer.on_viewer_draw)
        self._poll(handler=proximity_viewer.on_poll)

        contact = _ContactApplet(meshcat=self.meshcat)
        self._subscribe(channel="CONTACT_RESULTS",
                        message_type=lcmt_contact_results_for_viz,
                        handler=contact.on_contact_results)

        # Subscribe to all the point-cloud-related channels.
        point_cloud = _PointCloudApplet(meshcat=self.meshcat)
        self._subscribe_multichannel(regex="DRAKE_POINT_CLOUD.*",
                                     message_type=lcmt_point_cloud,
                                     handler=point_cloud.on_point_cloud)

        # Bookkeeping for automatic shutdown.
        self._last_poll = None
        self._last_active = None

    def _subscribe(self, channel, message_type, handler):
        """Subscribes the handler to the given channel, using message_type to
        pass in a decoded message object (not the raw bytes). The handler will
        only be called at some maximum frequency. Messages on the same channel
        that arrive too quickly will be discarded.
        """
        # Record this channel's type and handler.
        assert self._message_types.get(channel, message_type) == message_type
        self._message_types[channel] = message_type

        # A wrapper to discard `channel` information as it's not used in the
        # actual handler.
        def _multi_handler(*, channel, message):
            handler(message)
        self._message_handlers.setdefault(channel, []).append(_multi_handler)

        # Subscribe using an internal function that implements "last one wins".
        # It's important to service the LCM queue as frequently as possible:
        #  https://github.com/RobotLocomotion/drake/issues/15234
        #  https://github.com/lcm-proj/lcm/issues/345
        # However, if the sender is transmitting visualization messages at
        # a high rate (e.g., if a sim is running much faster than realtime),
        # then we should only pass some of them along to MeshCat to avoid
        # flooding it. The handler merely records the message data; we'll
        # pass it along to MeshCat using our `self._should_update()` timer.
        def _on_message(data):
            self._message_pending_data[channel] = data
        self._lcm.Subscribe(channel=channel, handler=_on_message)

    def _subscribe_multichannel(self, regex, message_type, handler):
        """Subscribes the handler to a group of channels filtered by regex. How
        this function handles messages is the same as _subscribe() except that
        the channel name is only known when invoking the callback.
        """
        def _on_message(channel, data):
            if channel not in self._message_types:
                self._message_types[channel] = message_type
                self._message_handlers.setdefault(channel, []).append(handler)
            self._message_pending_data[channel] = data
        self._lcm.SubscribeMultichannel(regex=regex, handler=_on_message)

    def _poll(self, handler):
        self._poll_handlers.append(handler)

    def _invoke_poll(self):
        for function in self._poll_handlers:
            function()

    def _invoke_subscriptions(self):
        """Posts any unhandled messages to their handlers and clears the
        collection of unhandled messages.
        """
        for channel, data in self._message_pending_data.items():
            message = self._message_types[channel].decode(data)
            for function in self._message_handlers[channel]:
                function(channel=channel, message=message)

        self._message_pending_data.clear()

    def serve_forever(self, *, idle_timeout=None):
        """Runs indefinitely, forwarding LCM => MeshCat messages.

        If provided, the optional idle_timeout must be strictly positive and
        this loop will sys.exit after that many seconds without any websocket
        connections.
        """
        while True:
            self._lcm.HandleSubscriptions(timeout_millis=1000)
            if not self._should_update():
                continue
            self._invoke_subscriptions()
            self._invoke_poll()
            self.meshcat.Flush()
            self._check_for_shutdown(idle_timeout=idle_timeout)

    def _should_update(self):
        """Posts LCM-driven updates to MeshCat no faster than 40 Hz."""
        now = time.time()
        update_period = 0.025  # 40 Hz
        remaining = update_period - (now - self._last_update_time)
        if remaining > 0.0:
            return False
        else:
            self._last_update_time = now
            return True

    def _check_for_shutdown(self, *, idle_timeout):
        # Allow the user to opt-out of the timeout feature.
        if idle_timeout is None:
            return
        assert idle_timeout > 0.0

        # One-time initialization.
        now = time.time()
        if self._last_active is None:
            self._last_active = now
            return

        # Only check once every 5 seconds.
        if (self._last_poll is not None) and (now < self._last_poll + 5.0):
            return
        self._last_poll = now

        # Check to see if any browser client(s) are connected.
        if self.meshcat.GetNumActiveConnections() > 0:
            self._last_active = now
            return

        # In case we are idle for too long, exit automatically.
        if now > self._last_active + idle_timeout:
            _logger.info("Meldis is exiting now; no browser was connected for"
                         f" >{idle_timeout} seconds")
            sys.exit(1)
