"""
Provides utilities for communicating with the browser-based visualization
package, Meshcat:
      https://github.com/rdeits/meshcat
"""
from __future__ import print_function
import argparse
import math
import webbrowser

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
            [source_name, frame_name] = link.name.split("::")

            for j in range(link.num_geom):
                geom = link.geom[j]
                # MultibodyPlant currenly sets alpha=0 to make collision
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
            [source_name, frame_name] = pose_bundle.get_name(frame_i)\
                .split("::")
            model_id = pose_bundle.get_model_instance_id(frame_i)
            # The MBP parsers only register the plant as a nameless source.
            # TODO(russt): Use a more textual naming convention here?
            self.vis[self.prefix][source_name][str(model_id)][frame_name]\
                .set_transform(pose_bundle.get_pose(frame_i).matrix())
