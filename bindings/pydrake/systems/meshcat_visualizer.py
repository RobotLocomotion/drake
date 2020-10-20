"""
Provides utilities for communicating with the browser-based visualization
package, Meshcat:
      https://github.com/rdeits/meshcat
"""
import argparse
import math
import os
import uuid
import warnings
import webbrowser

import numpy as np

from drake import lcmt_viewer_load_robot
from pydrake.common.value import AbstractValue
from pydrake.common.eigen_geometry import Quaternion, Isometry3
from pydrake.geometry import DispatchLoadMessage, SceneGraph
from pydrake.lcm import DrakeLcm, Subscriber
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.framework import LeafSystem, PublishEvent, TriggerType
from pydrake.systems.rendering import PoseBundle
from pydrake.multibody.plant import ContactResults
import pydrake.perception as mut

# TODO(eric.cousineau): Move this back to "third party" import positions
# if/when PyCQA/pycodestyle#834 lands and is incorporated.
with warnings.catch_warnings():
    warnings.filterwarnings(
        "ignore", category=ImportWarning,
        message="can't resolve package from __spec__")
    # TODO(SeanCurtis-TRI): Meshcat modified itself from a conditional
    # import of IPython to an unconditional import. IPython eventually
    # depends on imp. If the dependency becomes conditional or the
    # ultimate dependency upgrades from imp to importlib, this can be
    # removed.
    warnings.filterwarnings(
        "ignore", message="the imp module is deprecated",
        category=DeprecationWarning
    )
    import meshcat
import meshcat.geometry as g  # noqa
import meshcat.transformations as tf  # noqa
from meshcat.animation import Animation

_DEFAULT_PUBLISH_PERIOD = 1 / 30.


def _convert_mesh(geom):
    # Given a LCM geometry message, forms a meshcat geometry and material
    # for that geometry, as well as a local tf to the meshcat geometry
    # that matches the LCM geometry.
    # For MESH geometry, this function checks if a texture file exists next
    # to the mesh file, and uses that to provide the material information if
    # present. For BOX, SPHERE, CYLINDER geometry as well as MESH geometry
    # not satisfying the above, this function uses the geom.color field to
    # create a flat color for the object. In the case of other geometry types,
    # both fields are returned as None.
    meshcat_geom = None
    material = None
    element_local_tf = RigidTransform(
        RotationMatrix(Quaternion(geom.quaternion)),
        geom.position).GetAsMatrix4()

    # Short-circuit if the geometry scale is invalid.
    # (All uses of float data should be strictly positive:
    # edge lengths for boxes, radius and length for
    # spheres and cylinders, and scaling for meshes.)
    if not all([x > 0 for x in geom.float_data]):
        return meshcat_geom, material, element_local_tf

    if geom.type == geom.BOX:
        assert geom.num_float_data == 3
        meshcat_geom = meshcat.geometry.Box(geom.float_data)
    elif geom.type == geom.SPHERE:
        assert geom.num_float_data == 1
        meshcat_geom = meshcat.geometry.Sphere(geom.float_data[0])
    elif geom.type == geom.CYLINDER:
        assert geom.num_float_data == 2
        meshcat_geom = meshcat.geometry.Cylinder(
            geom.float_data[1],
            geom.float_data[0])
        # In Drake, cylinders are along +z
        # In meshcat, cylinders are along +y
        # Rotate to fix this misalignment
        extra_rotation = tf.rotation_matrix(
            math.pi/2., [1, 0, 0])
        element_local_tf[0:3, 0:3] = (
            element_local_tf[0:3, 0:3].dot(
                extra_rotation[0:3, 0:3]))
    elif geom.type == geom.MESH:
        meshcat_geom = meshcat.geometry.ObjMeshGeometry.from_file(
            geom.string_data[0:-3] + "obj")
        # Handle scaling.
        # TODO(gizatt): See meshcat-python#40 for incorporating scale as a
        # field rather than a matrix multiplication.
        scale = geom.float_data[:3]
        element_local_tf[:3, :3] = element_local_tf[:3, :3].dot(np.diag(scale))
        # Attempt to find a texture for the object by looking for an
        # identically-named *.png next to the model.
        # TODO(gizatt): Support .MTLs and prefer them over png, since they're
        # both more expressive and more standard.
        # TODO(gizatt): In the long term, this kind of material information
        # should be gleaned from the SceneGraph constituents themselves, so
        # that we visualize what the simulation is *actually* reasoning about
        # rather than what files happen to be present.
        candidate_texture_path_png = geom.string_data[0:-3] + "png"
        if os.path.exists(candidate_texture_path_png):
            material = meshcat.geometry.MeshLambertMaterial(
                map=meshcat.geometry.ImageTexture(
                    image=meshcat.geometry.PngImage.from_file(
                        candidate_texture_path_png)))
    else:
        print("UNSUPPORTED GEOMETRY TYPE {} IGNORED".format(
              geom.type))
        return meshcat_geom, material

    if material is None:
        def rgb_2_hex(rgb):
            # Turn a list of R,G,B elements (any indexable list
            # of >= 3 elements will work), where each element is
            # specified on range [0., 1.], into the equivalent
            # 24-bit value 0xRRGGBB.
            val = 0
            for i in range(3):
                val += (256**(2 - i)) * int(255 * rgb[i])
            return val
        material = meshcat.geometry.MeshLambertMaterial(
            color=rgb_2_hex(geom.color[:3]),
            transparent=geom.color[3] != 1.,
            opacity=geom.color[3])
    return meshcat_geom, material, element_local_tf


def AddTriad(vis, name, prefix, length=1., radius=0.04, opacity=1.):
    """
    Initializes coordinate axes of a frame T. The x-axis is drawn red,
    y-axis green and z-axis blue. The axes point in +x, +y and +z directions,
    respectively.
    TODO(pangtao22): replace cylinder primitives with ArrowHelper or AxesHelper
    one they are bound in meshcat-python.
    Args:
        vis: a meshcat.Visualizer object.
        name: (string) the name of the triad in meshcat.
        prefix: (string) name of the node in the meshcat tree to which this
            triad is added.
        length: the length of each axis in meters.
        radius: the radius of each axis in meters.
        opacity: the opacity of the coordinate axes, between 0 and 1.
    """
    delta_xyz = np.array([[length / 2, 0, 0],
                          [0, length / 2, 0],
                          [0, 0, length / 2]])

    axes_name = ['x', 'y', 'z']
    colors = [0xff0000, 0x00ff00, 0x0000ff]
    rotation_axes = [[0, 0, 1], [0, 1, 0], [1, 0, 0]]

    for i in range(3):
        material = meshcat.geometry.MeshLambertMaterial(
            color=colors[i], opacity=opacity)
        vis[prefix][name][axes_name[i]].set_object(
            meshcat.geometry.Cylinder(length, radius), material)
        X = meshcat.transformations.rotation_matrix(
            np.pi/2, rotation_axes[i])
        X[0:3, 3] = delta_xyz[i]
        vis[prefix][name][axes_name[i]].set_transform(X)


class MeshcatVisualizer(LeafSystem):
    """
    MeshcatVisualizer is a System block that connects to the pose bundle output
    port of a SceneGraph and visualizes the scene in Meshcat.

    The most common workflow would be to run::

        bazel run @meshcat_python//:meshcat-server

    in another terminal, open the url printed in that terminal in your
    browser, then to run drake apps (potentially many times) that publish to
    that default url.
    """

    @staticmethod
    def add_argparse_argument(parser):
        """
        Provides a common command-line interface for including meshcat support
        in a python executable.  Example::

            parser = argparse.ArgumentParser(description=__doc__)
            ...
            MeshcatVisualizer.add_argparse_argument(parser)
            ...
            args = parser.parse_args()
            ...
            if args.meshcat:
                meshcat = builder.AddSystem(MeshcatVisualizer(
                        scene_graph, zmq_url=args.meshcat,
                        open_browser=args.open_browser))
        """
        parser.add_argument(
            "--meshcat", nargs='?', metavar='zmq_url', const="new",
            default=None,
            help="Enable visualization with meshcat. If no zmq_url is "
                 "specified, a meshcat-server will be started as a "
                 "subprocess.  Use e.g. --meshcat=tcp://127.0.0.1:6000 to "
                 "connect to an existing meshcat-server at the specified "
                 "url.  Use --meshcat=default to connect to the "
                 "meshcat-server running at the default url.")

        parser.add_argument(
            "--open_browser", action='store_true', default=False,
            help="Open a browser when creating a new meshcat-server.")

    def __init__(self,
                 scene_graph,
                 draw_period=_DEFAULT_PUBLISH_PERIOD,
                 prefix="drake",
                 zmq_url="default",
                 open_browser=None,
                 frames_to_draw={},
                 frames_opacity=1.,
                 axis_length=0.15,
                 axis_radius=0.006,
                 delete_prefix_on_load=True,
                 **kwargs):
        """
        Args:
            scene_graph: A SceneGraph object.
            draw_period: The rate at which this class publishes to the
                visualizer.
            prefix: Appears as the root of the tree structure in the meshcat
                data structure
            zmq_url: Optionally set a url to connect to the visualizer.
                Use ``zmp_url="default"`` to the value obtained by running a
                single ``meshcat-server`` in another terminal.
                Use ``zmp_url=None`` or ``zmq_url="new"`` to start a new server
                (as a child of this process); a new web browser will be opened
                (the url will also be printed to the console).
                Use e.g. ``zmq_url="tcp://127.0.0.1:6000"`` to specify a
                specific address.
            open_browser: Set to True to open the meshcat browser url in your
                default web browser.  The default value of None will open the
                browser iff a new meshcat server is created as a subprocess.
                Set to False to disable this.
            frames_to_draw: a dictionary describing which body frames to draw.
                It is keyed on the names of model instances, and the values are
                sets of the names of the bodies whose body frames are shown.
                For example, if we want to draw the frames of body "A" and
                "B" in model instance "1", then frames_to_draw is
                    {"1": {"A", "B"}}.
            frames_opacity, axis_length and axis_radius are the opacity, length
                and radius of the coordinate axes to be drawn.
            delete_prefix_on_load: Specifies whether we should delete the
                visualizer path associated with prefix on our load
                initialization event.  True ensures a clean slate for every
                simulation.  False allows for the possibility of caching object
                meshes on the zmqserver/clients, to avoid repeatedly
                downloading meshes over the websockets link, but will cause
                geometry from previous simulations to remain in the scene.  You
                may call ``delete_prefix()`` manually to clear the scene.

        Additional kwargs will be passed to the MeshcatVisualizer constructor.
        Note:
            This call will not return until it connects to the
            ``meshcat-server``.
        """
        LeafSystem.__init__(self)

        self.set_name('meshcat_visualizer')
        self.DeclarePeriodicPublish(draw_period, 0.0)
        self.draw_period = draw_period
        self._delete_prefix_on_load = delete_prefix_on_load

        # Recording.
        self._is_recording = False
        self.reset_recording()

        # Pose bundle (from SceneGraph) input port.
        # TODO(tehbelinda): Rename the `lcm_visualization` port to match
        # SceneGraph once its output port has been updated. See #12214.
        self.DeclareAbstractInputPort("lcm_visualization",
                                      AbstractValue.Make(PoseBundle(0)))

        if zmq_url == "default":
            zmq_url = "tcp://127.0.0.1:6000"
        elif zmq_url == "new":
            zmq_url = None

        if zmq_url is None and open_browser is None:
            open_browser = True

        # Set up meshcat.
        self.prefix = prefix
        if zmq_url is not None:
            print("Connecting to meshcat-server at zmq_url=" + zmq_url + "...")
        self.vis = meshcat.Visualizer(zmq_url=zmq_url, **kwargs)
        print("Connected to meshcat-server.")
        self._scene_graph = scene_graph

        # Set background color (to match drake-visualizer).
        self.vis['/Background'].set_property("top_color", [242, 242, 255])
        self.vis['/Background'].set_property("bottom_color", [77, 77, 89])

        if open_browser:
            webbrowser.open(self.vis.url())

        def on_initialize(context, event):
            self.load()

        self.DeclareInitializationEvent(
            event=PublishEvent(
                trigger_type=TriggerType.kInitialization,
                callback=on_initialize))

        # drawing coordinate frames
        self.frames_to_draw = frames_to_draw
        self.frames_opacity = frames_opacity
        self.axis_length = axis_length
        self.axis_radius = axis_radius

    def set_planar_viewpoint(
        self, camera_position=[0, -1, 0], camera_focus=[0, 0, 0], xmin=-1,
            xmax=1, ymin=-1, ymax=1):
        """
        Sets meshcat to use an orthographic projection with locked out orbit
        controls, and turns off the background, axes, and grid.  This allows
        meshcat to play a role similar to PlanarSceneGraphVisualizer.

        Args:
            camera_position: a 3 element vector specifying the initial xyz
                location of the camera.
            camera_focus: a 3 element vector specifying the focal point of the
                camera.
            xmin, xmax, ymin, ymax: Scalars specifying the initial view
                coordinates of the orthographic camera.  (Note that xmax is
                actually ignored because the view is scaled to fit the size of
                the browser window).

        Note: The orientation of the camera will be locked, but users can still
        pan and zoom interactively in the browser.
        """

        # TODO(russt): Figure out the proper set of camera transformations to
        # implement camera_focus.
        if np.any(camera_focus):
            warnings.warn("Non-zero camera_focus is not supported yet")

        # Set up orthographic camera.
        camera = g.OrthographicCamera(
            left=xmin, right=xmax, top=ymax, bottom=ymin, near=-1000, far=1000)
        self.vis['/Cameras/default/rotated'].set_object(camera)
        self.vis['/Cameras/default'].set_transform(
            RigidTransform(camera_position).GetAsMatrix4())

        # Lock the orbit controls.
        self.vis['/Cameras/default/rotated/<object>'].set_property(
            "position", [0, 0, 0])

        # Turn off background, axes, and grid.
        self.vis['/Background'].set_property("visible", False)
        self.vis['/Grid'].set_property("visible", False)
        self.vis['/Axes'].set_property("visible", False)

    def _parse_name(self, name):
        # Parse name, split on the first occurrence of `::` to get the source
        # name, and let the rest be the frame name. If `::` is not in name,
        # source name is "unnamed" and the frame name is `name`.
        # TODO(eric.cousineau): Remove name parsing once this is reimplemented
        # to use Shape introspection.
        delim = "::"
        if delim not in name:
            default_source = "unnamed"
            return default_source, name
        pos = name.index(delim)
        source_name = name[:pos]
        frame_name = name[pos + len(delim):]
        return source_name, frame_name

    def delete_prefix(self):
        """
        Manually delete the meshcat prefix path specified in the constructor.
        All objects/transforms at or below this path will be removed.  This
        effectively clears all previous visualizations.  You will need to call
        ``load()`` directly, or trigger the initialization event that occurs at
        the beginning of a simulation, to see any visualizations.
        """
        self.vis[self.prefix].delete()

    def load(self):
        """
        Loads ``meshcat`` visualization elements.

        Precondition:
            The ``scene_graph`` used to construct this object must be part of a
            fully constructed diagram (e.g. via ``DiagramBuilder.Build()``).
        """
        if self._delete_prefix_on_load:
            self.vis[self.prefix].delete()

        # Intercept load message via memq LCM.
        memq_lcm = DrakeLcm("memq://")
        memq_lcm_subscriber = Subscriber(
            lcm=memq_lcm,
            channel="DRAKE_VIEWER_LOAD_ROBOT",
            lcm_type=lcmt_viewer_load_robot)
        DispatchLoadMessage(self._scene_graph, memq_lcm)
        memq_lcm.HandleSubscriptions(0)
        assert memq_lcm_subscriber.count > 0
        load_robot_msg = memq_lcm_subscriber.message

        # Translate elements to `meshcat`.
        for i in range(load_robot_msg.num_links):
            link = load_robot_msg.link[i]
            [_, frame_name] = self._parse_name(link.name)

            for j in range(link.num_geom):
                geom = link.geom[j]
                # MultibodyPlant currently sets alpha=0 to make collision
                # geometry "invisible".  Ignore those geometries here.
                if geom.color[3] == 0:
                    continue

                meshcat_geom, material, element_local_tf = _convert_mesh(geom)
                if meshcat_geom is not None:
                    cur_vis = (
                        self.vis[self.prefix][str(link.robot_num)]
                        [frame_name][str(j)])
                    # Make the uuid's deterministic for mesh geometry, to
                    # support caching at the zmqserver.  This means that
                    # multiple (identical) geometries may have the same UUID,
                    # but testing suggests that meshcat + three.js are ok with
                    # it.
                    if isinstance(meshcat_geom, meshcat.geometry.MeshGeometry):
                        meshcat_geom.uuid = str(uuid.uuid5(
                            uuid.NAMESPACE_X500, meshcat_geom.contents))
                        material.uuid = str(uuid.uuid5(
                            uuid.NAMESPACE_X500, meshcat_geom.contents
                            + "material"))
                        mesh = meshcat.geometry.Mesh(meshcat_geom, material)
                        mesh.uuid = str(uuid.uuid5(
                            uuid.NAMESPACE_X500, meshcat_geom.contents
                            + "mesh"))
                        cur_vis.set_object(mesh)
                    else:
                        cur_vis.set_object(meshcat_geom, material)

                    cur_vis.set_transform(element_local_tf)

                # Draw the frames in self.frames_to_draw.
                if "::" in frame_name:
                    robot_name, link_name = self._parse_name(frame_name)
                else:
                    robot_name = "world"
                    link_name = frame_name
                if (robot_name in self.frames_to_draw.keys()
                        and link_name in self.frames_to_draw[robot_name]):
                    prefix = (self.prefix + '/' + str(link.robot_num) + '/'
                              + frame_name)
                    AddTriad(
                        self.vis,
                        name="frame",
                        prefix=prefix,
                        length=self.axis_length,
                        radius=self.axis_radius,
                        opacity=self.frames_opacity)

    def DoPublish(self, context, event):
        # TODO(russt): Change this to declare a periodic event with a
        # callback instead of overriding DoPublish, pending #9992.
        LeafSystem.DoPublish(self, context, event)

        pose_bundle = self.EvalAbstractInput(context, 0).get_value()

        for frame_i in range(pose_bundle.get_num_poses()):
            # SceneGraph currently sets the name in PoseBundle as
            #    "get_source_name::frame_name".
            [_, frame_name] = self._parse_name(
                pose_bundle.get_name(frame_i))
            model_id = pose_bundle.get_model_instance_id(frame_i)
            # The MBP parsers only register the plant as a nameless source.
            # TODO(russt): Use a more textual naming convention here?
            pose_matrix = pose_bundle.get_transform(frame_i)
            cur_vis = (
                self.vis[self.prefix][str(model_id)][frame_name])
            cur_vis.set_transform(pose_matrix.GetAsMatrix4())
            if self._is_recording:
                with self._animation.at_frame(
                        cur_vis, self._recording_frame_num) as cur_vis_frame:
                    cur_vis_frame.set_transform(pose_matrix.GetAsMatrix4())

        if self._is_recording:
            self._recording_frame_num += 1

    def start_recording(self):
        """
        Sets a flag to record future publish events as animation frames.
        """
        # Note: This syntax was chosen to match PyPlotVisualizer.
        self._is_recording = True

    def stop_recording(self):
        """
        Disables the recording of future publish events.
        """
        # Note: This syntax was chosen to match PyPlotVisualizer.
        self._is_recording = False

    def publish_recording(self, play=True, repetitions=1):
        """
        Publish any recorded animation to Meshcat.  Use the controls dialog
        in the browser to review it.

        Args:
            play: boolean that determines whether the animation will play
                automatically.
            repetitions: number of times that the animation should play.
        """
        self.vis.set_animation(self._animation, play=play,
                               repetitions=repetitions)

    def reset_recording(self):
        """
        Resets all recorded data.
        """
        # Note: This syntax was chosen to match PyPlotVisualizer.
        self._animation = Animation(default_framerate=1./self.draw_period)
        self._recording_frame_num = 0


class MeshcatContactVisualizer(LeafSystem):
    """
    MeshcatContactVisualizer is a System block that visualizes contact
    forces. It is connected to (1) the pose bundle output port of a SceneGraph,
    and (2) the contact results output port of the SceneGraph's associated
    MultibodyPlant.
    """
    # TODO(russt): I am currently drawing both (equal and opposite) vector for
    # each contact.  Consider taking an additional option in the constructor to
    # provide a body prioritization, e.g. so that forces would always point
    # *out* of the body with higher priority.

    def __init__(self,
                 meshcat_viz,
                 force_threshold=1e-2,
                 contact_force_scale=10,
                 plant=None,
                 contact_force_radius=0.01):
        """
        Args:
            meshcat_viz: A pydrake MeshcatVisualizer instance.
            force_threshold: contact forces whose norms are smaller than
                force_threshold are not displayed.
            contact_force_scale: a contact force with norm F (in Newtons) is
                displayed as a cylinder with length F/contact_force_scale
                (in meters).
            plant: the MultibodyPlant associated with meshcat_viz.scene_graph.
            contact_force_radius: sets the constant radius of the cylinder used
                to visualize the forces.
        """
        LeafSystem.__init__(self)
        assert plant is not None
        self._meshcat_viz = meshcat_viz
        self._force_threshold = force_threshold
        self._contact_force_scale = contact_force_scale
        self._plant = plant
        self._radius = contact_force_radius

        self.set_name('meshcat_contact_visualizer')
        self.DeclarePeriodicPublish(self._meshcat_viz.draw_period, 0.0)

        # Pose bundle (from SceneGraph) input port.
        self.DeclareAbstractInputPort("pose_bundle",
                                      AbstractValue.Make(PoseBundle(0)))
        # Contact results input port from MultibodyPlant
        self.DeclareAbstractInputPort(
            "contact_results", AbstractValue.Make(ContactResults()))

        # This system has undeclared states, see #4330.
        self._warned_pose_bundle_input_port_connected = False
        self._published_contacts = []

        # Zap any previous contact forces on this prefix
        vis = self._meshcat_viz.vis[self._meshcat_viz.prefix]["contact_forces"]
        vis.delete()

    def DoPublish(self, context, event):
        LeafSystem.DoPublish(self, context, event)

        if (not self._warned_pose_bundle_input_port_connected
                and self.get_input_port(0).HasValue(context)):
            warnings.warn(
                "The pose_bundle input port of MeshcatContactVisualizer is no "
                "longer used and will be removed after 2021-01-01.")
            self._warned_pose_bundle_input_port_connected = True

        contact_results = self.EvalAbstractInput(context, 1).get_value()

        vis = self._meshcat_viz.vis[self._meshcat_viz.prefix]["contact_forces"]
        contacts = []

        for i_contact in range(contact_results.num_point_pair_contacts()):
            contact_info = contact_results.point_pair_contact_info(i_contact)

            # Do not display small forces.
            force_norm = np.linalg.norm(contact_info.contact_force())
            if force_norm < self._force_threshold:
                continue

            point_pair = contact_info.point_pair()
            key = (point_pair.id_A.get_value(), point_pair.id_B.get_value())
            cvis = vis[str(key)]
            contacts.append(key)
            arrow_height = self._radius*2.0
            if key not in self._published_contacts:
                # New key, so create the geometry. Note: the height of the
                # cylinder is 2 and gets scaled to twice the contact force
                # length, because I am drawing both (equal and opposite)
                # forces.  Note also that meshcat (following three.js) puts
                # the height of the cylinder along the y axis.
                cvis["cylinder"].set_object(meshcat.geometry.Cylinder(
                    height=2.0, radius=self._radius),
                    meshcat.geometry.MeshLambertMaterial(color=0x33cc33))
                cvis["head"].set_object(meshcat.geometry.Cylinder(
                    height=arrow_height,
                    radiusTop=0, radiusBottom=self._radius*2.0),
                    meshcat.geometry.MeshLambertMaterial(color=0x00dd00))
                cvis["tail"].set_object(meshcat.geometry.Cylinder(
                    height=arrow_height,
                    radiusTop=self._radius*2.0, radiusBottom=0),
                    meshcat.geometry.MeshLambertMaterial(color=0x00dd00))

            height = force_norm/self._contact_force_scale
            cvis["cylinder"].set_transform(tf.scale_matrix(
                height, direction=[0, 1, 0]))
            cvis["head"].set_transform(tf.translation_matrix(
                [0, height + arrow_height/2.0, 0.0]))
            cvis["tail"].set_transform(tf.translation_matrix(
                [0, -height - arrow_height/2.0, 0.0]))

            # Frame C is located at the contact point, but with the world frame
            # orientation.
            if force_norm < 1e-6:
                X_CGeom = tf.identity_matrix()
            else:
                # Rotates [0,1,0] to contact_force/force_norm.
                angle_axis = np.cross(np.array([0, 1, 0]),
                                      contact_info.contact_force()/force_norm)
                X_CGeom = tf.rotation_matrix(
                    np.arcsin(np.linalg.norm(angle_axis)), angle_axis)
            X_WC = tf.translation_matrix(contact_info.contact_point())
            cvis.set_transform(X_WC @ X_CGeom)

        # We only delete any contact vectors that did not persist into this
        # publish.  It is tempting to just delete() the root branch at the
        # beginning of this publish, but this leads to visual artifacts
        # (flickering) in the browser.
        for key in set(self._published_contacts) - set(contacts):
            vis[str(key)].delete()

        self._published_contacts = contacts


def _get_native_visualizer(viz):
    # Resolve `viz` to a native `meshcat.Visualizer` instance. If `viz` is
    # a pydrake `MeshcatVisualizer`, return the native visualizer.
    if isinstance(viz, meshcat.Visualizer):
        return viz
    elif isinstance(viz, MeshcatVisualizer):
        return viz.vis
    else:
        raise ValueError(
            "Type {} is not {{meshcat.Visualizer, {}}}".format(
                type(viz).__name__, MeshcatVisualizer.__name__))


class MeshcatPointCloudVisualizer(LeafSystem):
    """
    MeshcatPointCloudVisualizer is a System block that visualizes a
    PointCloud in meshcat. The PointCloud:

    * Must have XYZ values. Assumed to be in point cloud frame, ``P``.
    * RGB values are optional; if provided, they must be on the range [0..255].

    An example using a pydrake MeshcatVisualizer::

        viz = builder.AddSystem(MeshcatVisualizer(scene_graph))
        pc_viz = builder.AddSystem(
            MeshcatPointCloudVisualizer(viz, viz.draw_period))

    Using a native meshcat.Visualizer::

        viz = meshcat.Visualizer()
        pc_viz = builder.AddSystem(MeshcatPointCloudVisualizer(viz))

    System YAML
        name: MeshcatPointCloudVisualizer
        input_ports:
        - point_cloud_P
    """

    def __init__(self, meshcat_viz, draw_period=_DEFAULT_PUBLISH_PERIOD,
                 name="point_cloud", X_WP=RigidTransform.Identity(),
                 default_rgb=[255., 255., 255.]):
        """
        Args:
            meshcat_viz: Either a native meshcat.Visualizer or a pydrake
                MeshcatVisualizer object.
            draw_period: The rate at which this class publishes to the
                visualizer.
            name: The string name of the meshcat object.
            X_WP: Pose of point cloud frame ``P`` in meshcat world frame ``W``.
                Default is identity.
            default_rgb: RGB value for published points if the PointCloud does
                not provide RGB values.
        """
        LeafSystem.__init__(self)

        self._meshcat_viz = _get_native_visualizer(meshcat_viz)
        self._X_WP = RigidTransform(X_WP)
        self._default_rgb = np.array(default_rgb)
        self._name = name

        self.set_name('meshcat_point_cloud_visualizer_' + name)
        self.DeclarePeriodicPublish(draw_period, 0.0)

        self.DeclareAbstractInputPort("point_cloud_P",
                                      AbstractValue.Make(mut.PointCloud()))

    def DoPublish(self, context, event):
        LeafSystem.DoPublish(self, context, event)
        point_cloud_P = self.EvalAbstractInput(context, 0).get_value()

        # `Q` is a point in the point cloud.
        p_PQs = point_cloud_P.xyzs()
        # Use only valid points.
        valid = np.logical_not(np.isnan(p_PQs))
        valid = np.all(valid, axis=0)  # Reduce along XYZ axis.
        p_PQs = p_PQs[:, valid]
        if point_cloud_P.has_rgbs():
            rgbs = point_cloud_P.rgbs()[:, valid]
        else:
            # Need manual broadcasting.
            count = p_PQs.shape[1]
            rgbs = np.tile(np.array([self._default_rgb]).T, (1, count))
        # pydrake `PointCloud.rgbs()` are on [0..255], while meshcat
        # `PointCloud` colors are on [0..1].
        rgbs = rgbs / 255.  # Do not use in-place so we can promote types.
        # Send to meshcat.
        self._meshcat_viz[self._name].set_object(g.PointCloud(p_PQs, rgbs))
        self._meshcat_viz[self._name].set_transform(self._X_WP.GetAsMatrix4())


def ConnectMeshcatVisualizer(builder, scene_graph, output_port=None, **kwargs):
    """Creates an instance of MeshcatVisualizer, adds it to the
    diagram, and wires the scene_graph pose bundle output port to the input
    port of the visualizer.  Provides an interface comparable to
    ConnectDrakeVisualizer.

    Args:
        builder: The diagram builder used to construct the Diagram.
        scene_graph: The SceneGraph in builder containing the geometry to be
            visualized.
        output_port: (optional) If not None, then output_port will be connected
            to the visualizer input port instead of the scene_graph.
            get_pose_bundle_output_port().  This is required, for instance,
            when the SceneGraph is inside a Diagram, and we must connect the
            exposed port to the visualizer instead of the original SceneGraph
            port.

        Additional kwargs are passed through to the MeshcatVisualizer
        constructor.

    Returns:
        The newly created MeshcatVisualizer object.
    """
    if output_port is None:
        output_port = scene_graph.get_pose_bundle_output_port()
    visualizer = builder.AddSystem(
        MeshcatVisualizer(scene_graph, **kwargs))
    builder.Connect(output_port, visualizer.get_input_port(0))
    return visualizer
