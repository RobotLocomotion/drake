"""
Provides utilities for communicating with the browser-based visualization
package, Meshcat:
      https://github.com/rdeits/meshcat
"""
import math

from pydrake.util.eigen_geometry import Quaternion
from pydrake.geometry import DispatchLoadMessage, SceneGraph
from pydrake.lcm import DrakeMockLcm
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.framework import LeafSystem, PortDataType

from drake import lcmt_viewer_load_robot

import meshcat
import meshcat.transformations as tf


def Rgb2Hex(rgb):
    """ Turn a list of R,G,B elements (any indexable list of >= 3 elements
    will work), where each element is specified on range [0., 1.], into the
    equivalent 24-bit value 0xRRGGBB. """
    val = 0
    for i in range(3):
        val += (256**(2 - i)) * int(255 * rgb[i])
    return val


class MeshcatVisualizer(LeafSystem):
    """
    MeshcatVisualizer is a System block that connects to the pose bundle output
    port of a SceneGraph and visualizes the scene in Meshcat.

    The most common workflow would be to run
    `bazel run @meshcat_python//:meshcat-server`
    in another terminal, open the url printed in that terminal in your
    browser, then to run drake apps (potentially many times) that publish to
    that default url.

    Note that you *must* manually call the load() method after the
    SceneGraph has been finalized.
    """

    def __init__(self,
                 scene_graph,
                 draw_period=0.033333,
                 prefix="drake",
                 zmq_url="tcp://127.0.0.1:6000"):
        """
        Args:
            scene_graph: A SceneGraph object.
            draw_period: The rate at which this class publishes to the
                visualizer.
            prefix: Appears as the root of the tree structure in the meshcat
                data structure
            zmq_url: Optionally set a non-default url to connect to the
                visualizer.  The default value is the url obtained by
                running `meshcat-server` in another terminal.  If
                zmp_url=None, then then a new server will be automatically
                started (as a child of this process).
        """
        LeafSystem.__init__(self)

        self.set_name('meshcat_visualizer')
        self._DeclarePeriodicPublish(draw_period, 0.0)

        # Pose bundle (from SceneGraph) input port.
        self._DeclareInputPort("lcm_visualization",
                               PortDataType.kAbstractValued, 0)

        # Set up meshcat.
        self.prefix = prefix
        print("Connecting to meshcat-server...")
        self.vis = meshcat.Visualizer(zmq_url=zmq_url)
        print("Connected.")
        self.vis[self.prefix].delete()
        self._scene_graph = scene_graph

    def load(self):
        """
        Loads `meshcat` visualization elements.
        @pre The `scene_graph` used to construct this object must be part of a
        fully constructed diagram (e.g. via `DiagramBuilder.Build()`).
        """
        # TODO(russt): Declare an initialization event to publish this
        # pending resolution of #9842.

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
                    print "UNSUPPORTED GEOMETRY TYPE ", \
                        geom.type, " IGNORED"
                    continue

                self.vis[self.prefix][source_name][str(link.robot_num)][
                    frame_name][str(j)]\
                    .set_object(meshcat_geom,
                                meshcat.geometry
                                .MeshLambertMaterial(
                                    color=Rgb2Hex(geom.color)))
                self.vis[self.prefix][source_name][str(link.robot_num)][
                    frame_name][str(j)].set_transform(element_local_tf)

    def _DoPublish(self, context, event):
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
