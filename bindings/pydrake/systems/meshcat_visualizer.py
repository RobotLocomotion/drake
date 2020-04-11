"""
Provides utilities for communicating with the browser-based visualization
package, Meshcat:
      https://github.com/rdeits/meshcat
"""
import argparse
import math
import os
import warnings
import webbrowser

import numpy as np

from drake import lcmt_viewer_load_robot
from pydrake.common.eigen_geometry import Quaternion, Isometry3
from pydrake.geometry import DispatchLoadMessage, SceneGraph
from pydrake.lcm import DrakeLcm, Subscriber
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.framework import (
    AbstractValue, LeafSystem, PublishEvent, TriggerType
)
from pydrake.systems.rendering import PoseBundle
from pydrake.multibody.plant import ContactResults
import pydrake.perception as mut

# TODO(eric.cousineau): Move this back to "third party" import positions
# if/when PyCQA/pycodestyle#834 lands and is incorporated.
with warnings.catch_warnings():
    warnings.filterwarnings(
        "ignore", category=ImportWarning,
        message="can't resolve package from __spec__")
    import meshcat
import meshcat.geometry as g  # noqa
import meshcat.transformations as tf  # noqa

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
                 axis_radius=0.006):
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
        Note:
            This call will not return until it connects to the
            ``meshcat-server``.
        """
        LeafSystem.__init__(self)

        self.set_name('meshcat_visualizer')
        self.DeclarePeriodicPublish(draw_period, 0.0)
        self.draw_period = draw_period

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
        self.vis = meshcat.Visualizer(zmq_url=zmq_url)
        print("Connected to meshcat-server.")
        self._scene_graph = scene_graph

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

    def _parse_name(self, name):
        # Parse name, split on the first (required) occurrence of `::` to get
        # the source name, and let the rest be the frame name.
        # TODO(eric.cousineau): Remove name parsing once #9128 is resolved.
        delim = "::"
        assert delim in name
        pos = name.index(delim)
        source_name = name[:pos]
        frame_name = name[pos + len(delim):]
        return source_name, frame_name

    def load(self):
        """
        Loads ``meshcat`` visualization elements.

        Precondition:
            The ``scene_graph`` used to construct this object must be part of a
            fully constructed diagram (e.g. via ``DiagramBuilder.Build()``).
        """
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
            [source_name, frame_name] = self._parse_name(link.name)

            for j in range(link.num_geom):
                geom = link.geom[j]
                # MultibodyPlant currently sets alpha=0 to make collision
                # geometry "invisible".  Ignore those geometries here.
                if geom.color[3] == 0:
                    continue

                meshcat_geom, material, element_local_tf = _convert_mesh(geom)
                if meshcat_geom is not None:
                    cur_vis = (
                        self.vis[self.prefix][source_name][str(link.robot_num)]
                        [frame_name][str(j)])
                    cur_vis.set_object(meshcat_geom, material)
                    cur_vis.set_transform(element_local_tf)

                # Draw the frames in self.frames_to_draw.
                if "::" in frame_name:
                    robot_name, link_name = self._parse_name(frame_name)
                else:
                    robot_name = "world"
                    link_name = frame_name
                if (robot_name in self.frames_to_draw.keys() and
                        link_name in self.frames_to_draw[robot_name]):
                    prefix = (self.prefix + '/' + source_name + '/' +
                              str(link.robot_num) + '/' + frame_name)
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
            [source_name, frame_name] = self._parse_name(
                pose_bundle.get_name(frame_i))
            model_id = pose_bundle.get_model_instance_id(frame_i)
            # The MBP parsers only register the plant as a nameless source.
            # TODO(russt): Use a more textual naming convention here?
            pose_matrix = pose_bundle.get_pose(frame_i)
            self.vis[self.prefix][source_name][str(model_id)][frame_name]\
                .set_transform(pose_matrix.matrix())


class MeshcatContactVisualizer(LeafSystem):
    """
    MeshcatContactVisualizer is a System block that visualizes contact
    forces. It is connected to (1) the pose bundle output port of a SceneGraph,
    and (2) the contact results output port of the SceneGraph's associated
    MultibodyPlant.
    """

    class _ContactState(object):
        def __init__(self, key, needs_pruning, info, p_BC):
            # Key for use with meshcat.
            self.key = key
            # Implies contact should be pruned / removed.
            self.needs_pruning = needs_pruning
            # ContactInfo instance.
            self.info = info
            # Position of contact `C` w.r.t. body `B`.
            self.p_BC = p_BC

    def __init__(self,
                 meshcat_viz,
                 force_threshold=1e-2,
                 contact_force_scale=10,
                 plant=None):
        """
        Args:
            meshcat_viz: A pydrake MeshcatVisualizer instance.
            force_threshold: contact forces whose norms are smaller than
                force_threshold are not displayed.
            contact_force_scale: a contact force with norm F (in Newtons) is
                displayed as a cylinder with length F/contact_force_scale
                (in meters).
            plant: the MultibodyPlant associated with meshcat_viz.scene_graph.
        """
        LeafSystem.__init__(self)
        assert plant is not None
        self._meshcat_viz = meshcat_viz
        self._force_threshold = force_threshold
        self._contact_force_scale = contact_force_scale
        self._plant = plant

        self.set_name('meshcat_contact_visualizer')
        self.DeclarePeriodicPublish(self._meshcat_viz.draw_period, 0.0)
        # Pose bundle (from SceneGraph) input port.
        self.DeclareAbstractInputPort("pose_bundle",
                                      AbstractValue.Make(PoseBundle(0)))
        # Contact results input port from MultibodyPlant
        self.DeclareAbstractInputPort(
            "contact_results", AbstractValue.Make(ContactResults()))

        # Make force cylinders smaller at initialization.
        self._force_cylinder_radial_scale = 1.
        self._force_cylinder_longitudinal_scale = 100.

        # This system has undeclared states, see #4330.
        # - All contacts (previous and current), of type `_ContactState`.
        self._contacts = []
        # - Unique key for contacts in meshcat.
        self._contact_key_counter = 0
        # - Previous time at which contact was published.
        self._t_previous = 0.

    def DoPublish(self, context, event):
        LeafSystem.DoPublish(self, context, event)
        pose_bundle = self.EvalAbstractInput(context, 0).get_value()
        X_WB_map = self._get_pose_map(pose_bundle)
        self._draw_contact_forces(context, X_WB_map)

    def _get_pose_map(self, pose_bundle):
        # - Stores poses of all bodies in self._plant and its associated plant.
        # Created via `_store_pose_map`, keyed by
        # (int(body.model_index()), body.name()).
        X_WB_map = dict()
        for frame_i in range(pose_bundle.get_num_poses()):
            # SceneGraph currently sets the name in PoseBundle as
            # `{source_name}::{frame_name}`.
            (source_name, frame_name) = self._meshcat_viz._parse_name(
                pose_bundle.get_name(frame_i))
            model_instance = pose_bundle.get_model_instance_id(frame_i)
            pose_matrix = pose_bundle.get_pose(frame_i)
            _, frame_name = frame_name.split("::")
            key = (model_instance, frame_name)
            X_WB_map[key] = pose_matrix
        return X_WB_map

    def _find_duplicate_contact(self, new, dt):
        # Return first stored contact that is close enough, or None if contact
        # is new.
        assert isinstance(new, self._ContactState)
        for old in self._contacts:
            # Use order-insensitive comparison using `set`s.
            old_bodies = {int(old.info.bodyA_index()),
                          int(old.info.bodyB_index())}
            new_bodies = {int(new.info.bodyA_index()),
                          int(new.info.bodyB_index())}
            if old_bodies == new_bodies:
                # Reaching here means that `old` and `new`
                # describe contact between the same pair of bodies.
                v = np.sqrt(old.info.separation_speed()**2 +
                            old.info.slip_speed()**2)
                if np.linalg.norm(new.p_BC - old.p_BC) < v * dt:
                    old.info = new.info
                    old.p_BC = new.p_BC
                    return old
        return None

    def _draw_contact_forces(self, context, X_WB_map):
        contact_results = self.EvalAbstractInput(context, 1).get_value()
        t = context.get_time()

        # First, set all existing contacts to be pruned.
        for contact in self._contacts:
            contact.needs_pruning = True

        # Check if every element in contact_results is already in
        #   self._contacts.
        # If True, update the magnitude and location of
        #   the _ContactState in self._contacts.
        # If False, add the new contact to self._contacts
        vis = self._meshcat_viz.vis
        prefix = self._meshcat_viz.prefix
        for i_contact in range(contact_results.num_point_pair_contacts()):
            contact_info_i = contact_results.point_pair_contact_info(i_contact)

            # Do not display small forces.
            force_norm = np.linalg.norm(contact_info_i.contact_force())
            if force_norm < self._force_threshold:
                continue

            # contact point in frame B
            bodyB = self._plant.get_body(contact_info_i.bodyB_index())
            X_WB_key = (int(bodyB.model_instance()), bodyB.name())
            X_WB = X_WB_map[X_WB_key]
            p_BC = X_WB.inverse().multiply(contact_info_i.contact_point())
            new_contact = self._ContactState(
                key=str(self._contact_key_counter), needs_pruning=False,
                info=contact_info_i, p_BC=p_BC)

            contact = self._find_duplicate_contact(
                new_contact, dt=t - self._t_previous)
            if contact is None:
                # contact is new
                self._contacts.append(new_contact)
                self._contact_key_counter += 1
                # create cylinders with small radius.
                vis[prefix]["contact_forces"][new_contact.key].set_object(
                    meshcat.geometry.Cylinder(
                        height=1. / self._force_cylinder_longitudinal_scale,
                        radius=0.01 / self._force_cylinder_radial_scale),
                    meshcat.geometry.MeshLambertMaterial(color=0xff0000))
            else:
                # contact is not new, but it's valid.
                contact.needs_pruning = False

        # Prune old contact forces
        for contact in list(self._contacts):
            if contact.needs_pruning:
                self._contacts.remove(contact)
                vis[prefix]["contact_forces"][contact.key].delete()

        # visualize all valid contact forces
        for contact in self._contacts:
            # Compute pose of contact cylinder `C` in world frame `W`.
            R = np.zeros((3, 3))
            magnitude = np.linalg.norm(contact.info.contact_force())
            y = contact.info.contact_force() / magnitude
            R[:, 1] = y
            R[:, 0] = [0, -y[2], y[1]]
            R[:, 2] = np.cross(R[:, 0], y)
            X_WC = np.eye(4)
            X_WC[0:3, 0:3] = R
            X_WC[0:3, 3] = contact.info.contact_point()
            # Scale cylinder
            visual_magnitude = self._get_visual_magnitude(magnitude)
            T_scale = tf.translation_matrix(
                [0, visual_magnitude / 2, 0])
            T_scale[1, 1] = \
                visual_magnitude * self._force_cylinder_longitudinal_scale
            # - "expand" cylinders to a visible size.
            T_scale[0, 0] *= self._force_cylinder_radial_scale
            T_scale[2, 2] *= self._force_cylinder_radial_scale
            # Publish.
            vis[prefix]["contact_forces"][contact.key].set_transform(
                X_WC.dot(T_scale))

        # update time
        self._t_previous = t

    def _get_visual_magnitude(self, magnitude):
        return magnitude / self._contact_force_scale


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

    * Must have XYZ values. Assumed to be in point cloud frmae, ``P``.
    * RGB values are optional; if provided, they must be on the range [0..255].

    An example using a pydrake MeshcatVisualizer::

        viz = builder.AddSystem(MeshcatVisualizer(scene_graph))
        pc_viz = builder.AddSystem(
            MeshcatPointCloudVisualizer(viz, viz.draw_period))

    Using a native meshcat.Visualizer::

        viz = meshcat.Visualizer()
        pc_viz = builder.AddSystem(MeshcatPointCloudVisualizer(viz))

    System ports::

        @system{
            @input_port{point_cloud_P},
        }
    """

    def __init__(self, meshcat_viz, draw_period=_DEFAULT_PUBLISH_PERIOD,
                 name="point_cloud", X_WP=Isometry3.Identity(),
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
        self._X_WP = X_WP
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
        self._meshcat_viz[self._name].set_transform(self._X_WP.matrix())
