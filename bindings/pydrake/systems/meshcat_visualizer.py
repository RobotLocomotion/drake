"""
Provides utilities for communicating with the browser-based visualization
package, Meshcat:
      https://github.com/rdeits/meshcat
"""
from __future__ import print_function
import argparse
import math
import webbrowser
import numpy as np

import meshcat
import meshcat.transformations as tf

from drake import lcmt_viewer_load_robot
from pydrake.util.eigen_geometry import Quaternion
from pydrake.geometry import DispatchLoadMessage, SceneGraph
from pydrake.lcm import DrakeMockLcm
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.framework import (
    AbstractValue, LeafSystem, PublishEvent, TriggerType
)
from pydrake.systems.rendering import PoseBundle
from pydrake.multibody.multibody_tree.multibody_plant import ContactResults


class MeshcatVisualizer(LeafSystem):
    """
    MeshcatVisualizer is a System block that connects to the pose bundle output
    port of a SceneGraph and visualizes the scene in Meshcat.

    The most common workflow would be to run
    `bazel run @meshcat_python//:meshcat-server`
    in another terminal, open the url printed in that terminal in your
    browser, then to run drake apps (potentially many times) that publish to
    that default url.
    """

    @staticmethod
    def add_argparse_argument(parser):
        """
        Provides a common command-line interface for including meshcat support
        in a python executable.  Example:
            parser = argparse.ArgumentParser(description=__doc__)
            ...
            MeshcatVisualizer.add_argparse_argument(parser)
            ...
            args = parser.parse_args()

            ...

            if args.meshcat:
                meshcat = builder.AddSystem(MeshcatVisualizer(
                        scene_graph, zmq_url=args.meshcat))
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

    def __init__(self,
                 scene_graph,
                 draw_period=0.033333,
                 prefix="drake",
                 zmq_url="default",
                 open_browser=None):
        """
        Args:
            scene_graph: A SceneGraph object.
            draw_period: The rate at which this class publishes to the
                visualizer.
            prefix: Appears as the root of the tree structure in the meshcat
                data structure
            zmq_url: Optionally set a url to connect to the visualizer.
                Use zmp_url="default" to the value obtained by running a
                single `meshcat-server` in another terminal.
                Use zmp_url=None or zmq_url="new" to start a new server (as a
                child of this process); a new web browser will be opened (the
                url will also be printed to the console).
                Use e.g. zmq_url="tcp://127.0.0.1:6000" to specify a
                specific address.
            open_browser: Set to True to open the meshcat browser url in your
                default web browser.  The default value of None will open the
                browser iff a new meshcat server is created as a subprocess.
                Set to False to disable this.

        Note: This call will not return until it connects to the
              meshcat-server.
        """
        LeafSystem.__init__(self)

        self.set_name('meshcat_visualizer')
        self._DeclarePeriodicPublish(draw_period, 0.0)
        self.draw_period = draw_period

        # Pose bundle (from SceneGraph) input port.
        self._DeclareAbstractInputPort("lcm_visualization",
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

        self._DeclareInitializationEvent(
            event=PublishEvent(
                trigger_type=TriggerType.kInitialization,
                callback=on_initialize))

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
        Loads `meshcat` visualization elements.

        @pre The `scene_graph` used to construct this object must be part of a
        fully constructed diagram (e.g. via `DiagramBuilder.Build()`).
        """
        self.vis[self.prefix].delete()

        # Intercept load message via mock LCM.
        mock_lcm = DrakeMockLcm()
        DispatchLoadMessage(self._scene_graph, mock_lcm)
        load_robot_msg = lcmt_viewer_load_robot.decode(
            mock_lcm.get_last_published_message("DRAKE_VIEWER_LOAD_ROBOT"))
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
                    element_local_tf[0:3, 0:3] = \
                        element_local_tf[0:3, 0:3].dot(
                            extra_rotation[0:3, 0:3])
                elif geom.type == geom.MESH:
                    meshcat_geom = \
                        meshcat.geometry.ObjMeshGeometry.from_file(
                            geom.string_data[0:-3] + "obj")
                else:
                    print("UNSUPPORTED GEOMETRY TYPE {} IGNORED".format(
                          geom.type))
                    continue

                # Turn a list of R,G,B elements (any indexable list of >= 3
                # elements will work), where each element is specified on range
                # [0., 1.], into the equivalent 24-bit value 0xRRGGBB.
                def Rgb2Hex(rgb):
                    val = 0
                    for i in range(3):
                        val += (256**(2 - i)) * int(255 * rgb[i])
                    return val

                self.vis[self.prefix][source_name][str(link.robot_num)][
                    frame_name][str(j)]\
                    .set_object(meshcat_geom,
                                meshcat.geometry
                                .MeshLambertMaterial(
                                    color=Rgb2Hex(geom.color)))
                self.vis[self.prefix][source_name][str(link.robot_num)][
                    frame_name][str(j)].set_transform(element_local_tf)

    def _DoPublish(self, context, event):
        # TODO(russt): Change this to declare a periodic event with a
        # callback instead of overriding _DoPublish, pending #9992.
        LeafSystem._DoPublish(self, context, event)

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
    forces. It is connected to
    1) the pose bundle output port of a SceneGraph, and
    2) the contact results output port of the SceneGraph's associated
        MultibodyPlant.
    """
    def __init__(self,
                 meshcat_viz,
                 force_threshold=0.,
                 contact_force_scale=10,
                 plant=None):
        """
        Args:
            meshcat_viz: a MeshcatVisualizer object.
            force_threshold: contact forces whose norms are smaller than
                force_threshold are not displayed.
            contact_force_scale: a contact force with norm F (in Newtons) is
                displayed as a cylinder with length F/contact_force_scale
                (in meters).
            plant: the MultibodyPlant associated with meshcat_viz.scene_graph.
        """
        LeafSystem.__init__(self)
        self.set_name('meshcat_contact_visualizer')
        self._DeclarePeriodicPublish(meshcat_viz.draw_period, 0.0)

        self.meshcat_viz = meshcat_viz
        self.force_threshold = force_threshold
        self.contact_force_scale = contact_force_scale
        assert not (plant is None)
        self.plant = plant

        # Pose bundle (from SceneGraph) input port.
        self._DeclareAbstractInputPort("lcm_visualization",
                                       AbstractValue.Make(PoseBundle(0)))

        # Contact results input port from MultiBodyPlant
        self._DeclareAbstractInputPort(
            "contact_results", AbstractValue.Make(ContactResults()))

        # A dictionary of contact_info objects.
        self.contact_info_dict = dict()

        # A dictionary of contact points in bodyB's body frame,
        # sharing the same keys as contact_info_dict.
        self.p_BC_dict = dict()

        # contact_idx is used as keys in contact_info_dic and p_BC_dict,
        # It is icremented whenever a there is a new contact.
        self.contact_idx = 0
        self.t_previous = 0.

        # body_pose_dict stores the poses of all bodies in self.plant
        #  and its associated scene_graph.
        # This dict is keyed by (int(body.model_index()), body.name()).
        # This dictionary is created to facilitate searching for body poses
        #  using the body's name and model instance index.
        # It must be updated before drawing contact forces.
        self.body_pose_dict = dict()

        # Make force cylinders smaller at initialization.
        self.force_cylinder_radial_scale = 1.
        self.force_cylinder_longitudinal_scale = 100.

    def _DoPublish(self, context, event):
        LeafSystem._DoPublish(self, context, event)

        pose_bundle = self.EvalAbstractInput(context, 0).get_value()

        for frame_i in range(pose_bundle.get_num_poses()):
            # SceneGraph currently sets the name in PoseBundle as
            #    "get_source_name::frame_name".
            [source_name, frame_name] = \
                self.meshcat_viz._parse_name(
                    pose_bundle.get_name(frame_i))
            model_id = pose_bundle.get_model_instance_id(frame_i)
            pose_matrix = pose_bundle.get_pose(frame_i)

            _, frame_name = frame_name.split("::")
            key = (int(model_id), frame_name)
            self.body_pose_dict[key] = pose_matrix

        self._draw_contact_forces(context)

    def _is_contact_new(self, contact_info, dt, p_BC):
        """
        contact_info: a contact_info object
        dt: elapsed simulation time since the last draw event.
        p_BC: the coordinate of contact point in bodyB frame.
        If contact_info is in self.contact_info_dict,
          return False and the key of the corresponding contact
          in self.contact_info_dict.
        """
        is_contact_new = True
        key_contact_dict = None
        for key, contact_info_i in self.contact_info_dict.iteritems():
            are_bodies_same1 =\
                contact_info_i.bodyA_index() == contact_info.bodyA_index()\
                and\
                contact_info_i.bodyB_index() == contact_info.bodyB_index()
            are_bodies_same2 = \
                contact_info_i.bodyA_index() == contact_info.bodyB_index()\
                and\
                contact_info_i.bodyB_index() == contact_info.bodyA_index()
            if are_bodies_same1 or are_bodies_same2:
                # Reaching here means that contact_info_i and contact_info
                # describe contact between the same pair of bodies.
                v = np.sqrt(contact_info_i.separation_speed()**2 +
                            contact_info_i.slip_speed()**2)

                if np.linalg.norm(p_BC - self.p_BC_dict[key]) < v * dt:
                    is_contact_new = False
                    key_contact_dict = key
                    break

        return is_contact_new, key_contact_dict

    def _draw_contact_forces(self, context):
        contact_results = self.EvalAbstractInput(context, 1).get_value()
        t = context.get_time()

        # First, set all existing contacts to be "invalid".
        is_contact_valid = dict()
        for key in self.contact_info_dict.keys():
            is_contact_valid[key] = False

        # Check if every element in contact_results is already in
        #   self.contact_info_dict.
        # If True, update the magnitude and location of
        #   the contact_info in self.contact_info_dict.
        # If False, add the new contact_info to self.contact_info_dict
        viz = self.meshcat_viz.vis
        prefix = self.meshcat_viz.prefix
        for i_contact in range(contact_results.num_contacts()):
            contact_info_i = contact_results.contact_info(i_contact)

            # Do not display small forces.
            force_norm = np.linalg.norm(contact_info_i.contact_force())
            if force_norm < self.force_threshold:
                continue

            # contact point in frame B
            bodyB = self.plant.get_body(contact_info_i.bodyB_index())
            bodyB_name = (int(bodyB.model_instance()), bodyB.name())

            X_WB = self.body_pose_dict[bodyB_name]
            p_BC = X_WB.inverse().multiply(contact_info_i.contact_point())

            dt = t - self.t_previous
            is_contact_new, key =\
                self._is_contact_new(contact_info_i, dt, p_BC)
            if is_contact_new:
                # contact is new
                new_key = str(self.contact_idx)
                is_contact_valid[new_key] = True
                self.contact_info_dict[new_key] = contact_info_i
                # create cylinders with small radius.
                viz[prefix]["contact_forces"][new_key].set_object(
                    meshcat.geometry.Cylinder(
                        height=1./self.force_cylinder_longitudinal_scale,
                        radius=0.01/self.force_cylinder_radial_scale),
                    meshcat.geometry.MeshLambertMaterial(color=0xff0000))
                # Every new contact has its contact point in bodyB frame stored
                # in self.p_BC.dict.
                self.p_BC_dict[new_key] = p_BC

                self.contact_idx += 1
            else:
                self.contact_info_dict[key] = contact_info_i
                is_contact_valid[key] = True

        # delete invalid contact forces
        for key, is_valid in is_contact_valid.iteritems():
            if not is_valid:
                viz[prefix]["contact_forces"][key].delete()
                del self.contact_info_dict[key]

        # visualize all valid contact forces, and delete invalid contact forces
        for key, contact_info in self.contact_info_dict.iteritems():
            R = np.zeros((3, 3))
            magnitude = np.linalg.norm(contact_info.contact_force())
            y = contact_info.contact_force()/magnitude
            R[:, 1] = y
            R[:, 0] = [0, -y[2], y[1]]
            R[:, 2] = np.cross(R[:, 0], y)

            # transform cylinder
            visual_magnitude = self._get_visual_magnitude(magnitude)
            T0 = tf.translation_matrix(
                [0, visual_magnitude / 2, 0])
            T0[1, 1] = \
                visual_magnitude * self.force_cylinder_longitudinal_scale
            # "expand" cylinders to a visible size.
            T0[0, 0] *= self.force_cylinder_radial_scale
            T0[2, 2] *= self.force_cylinder_radial_scale

            T1 = np.eye(4)
            T1[0:3, 0:3] = R
            T1[0:3, 3] = contact_info.contact_point()

            viz[prefix]["contact_forces"][key]\
                .set_transform(T1.dot(T0))

        # update t_previous
        self.t_previous = t

    def _get_visual_magnitude(self, magnitude):
        return magnitude / self.contact_force_scale
