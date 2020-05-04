import math
import warnings

import errno
import glob
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import os
with warnings.catch_warnings():  # noqa
    # N.B. We must suppress this to appease `all_test`.
    # TODO(eric.cousineau): Remove this once all supported platform ships
    # `scipy>=1.0.0` by default.
    warnings.simplefilter("ignore", ImportWarning)
    import scipy as sp
    from scipy import spatial

from drake import lcmt_viewer_load_robot
from pydrake.common.eigen_geometry import Quaternion
from pydrake.geometry import DispatchLoadMessage, ReadObjToSurfaceMesh
from pydrake.lcm import DrakeLcm, Subscriber
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.framework import AbstractValue
from pydrake.systems.pyplot_visualizer import PyPlotVisualizer
from pydrake.systems.rendering import PoseBundle


class PlanarSceneGraphVisualizer(PyPlotVisualizer):
    """
    Given a SceneGraph and a view plane, provides a view of the robot by
    projecting all geometry onto the view plane.

    This is intended to be used for robots that operate in the plane, but
    should render any robot approximately correctly. It has the following
    caveats:
    - z-ordering of objects is done based on the object centroid, which
    is not perfect for non-planar scenes.
    - Object geometry is projected onto the view plane, then a chull is taken,
    and finally that chull is drawn as a patch. Nonconvex geometry will thus be
    drawn incorrectly, and geometry with many vertices will slow down the
    visualizer.

     Specifics on view setup:

    T_VW specifies the 3x4 view projection matrix. For planar orthographic
    projection, use:
    [ <x axis select> x_axis_shift
      <y axis select> y_axis_shift
       0, 0, 0, 1]  % homogenizer

    e.g.

    [ 1 0 0 0.5
      0 1 0 0
      0 0 0 1].

    would give a top-down view (i.e squashing the z axis), and would shift
    things in the x axis positively by 0.5.

    T_VW can be any valid view projection matrix. If the bottom row is
    [0, 0, 0, 1], the view projection will be an orthographic projection.

    xlim and ylim don't technically provide extra functionality, but it's
    easier to keep handling scaling with xlim, ylim, and view plane selection
    and *maybe* offsetting with the projection matrix.
    """

    def __init__(self,
                 scene_graph,
                 draw_period=1./30,
                 T_VW=np.array([[1., 0., 0., 0.],
                                [0., 0., 1., 0.],
                                [0., 0., 0., 1.]]),
                 xlim=[-1., 1],
                 ylim=[-1, 1],
                 facecolor=[1, 1, 1],
                 use_random_colors=False,
                 substitute_collocated_mesh_files=True,
                 ax=None,
                 show=None):
        """
        Args:
            scene_graph: A SceneGraph object.
            draw_period: The rate at which this class publishes to the
                visualizer.
            T_VW: The view projection matrix from world to view coordinates.
            xlim: View limit into the scene.
            ylim: View limit into the scene.
            facecolor: Passed through to figure() and sets background color.
                Both color name strings and RGB triplets are allowed. Defaults
                to white.
            use_random_colors: If set to True, will render each body with a
                different color. (Multiple visual elements on the same body
                will be the same color.)
            substitute_collocated_mesh_files: If True, then a mesh file
                specified with an unsupported filename extension may be
                replaced by a file of the same base name in the same directory,
                but with a supported filename extension.  Currently only .obj
                files are supported.
            ax: If supplied, the visualizer will draw onto those axes instead
                of creating a new set of axes. The visualizer will still change
                the view range and figure size of those axes.
            show: Opens a window during initialization / publish iff True.
                Default is None, which implies show=True unless
                matplotlib.get_backend() is 'template'.
        """
        default_size = matplotlib.rcParams['figure.figsize']
        scalefactor = (ylim[1]-ylim[0]) / (xlim[1]-xlim[0])
        figsize = (default_size[0], default_size[0]*scalefactor)

        PyPlotVisualizer.__init__(self, facecolor=facecolor, figsize=figsize,
                                  ax=ax, draw_period=draw_period, show=show)
        self.set_name('planar_scenegraph_visualizer')

        self._scene_graph = scene_graph
        self._T_VW = T_VW

        # Pose bundle (from SceneGraph) input port.
        # TODO(tehbelinda): Rename the `lcm_visualization` port to match
        # SceneGraph once its output port has been updated. See #12214.
        self._pose_bundle_port = self.DeclareAbstractInputPort(
            "lcm_visualization", AbstractValue.Make(PoseBundle(0)))

        self.ax.axis('equal')
        self.ax.axis('off')

        # Achieve the desired view limits.
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        default_size = self.fig.get_size_inches()
        self.fig.set_size_inches(figsize[0], figsize[1])

        # Populate body patches.
        self._build_body_patches(use_random_colors,
                                 substitute_collocated_mesh_files)

        # Populate the body fill list -- which requires doing most of a draw
        # pass, but with an ax.fill() command to initialize the draw patches.
        # After initialization, we can then use in-place replacement of vertex
        # positions. The body fill list stores the ax patch objects in the
        # order they were spawned (i.e. by body, and then by order of view_
        # patches). Drawing the tree should update them by iterating over
        # bodies and patches in the same order.
        self._body_fill_dict = {}
        X_WB_initial = RigidTransform.Identity()
        for full_name in self._patch_Blist.keys():
            patch_Wlist, view_colors = self._get_view_patches(full_name,
                                                              X_WB_initial)
            self._body_fill_dict[full_name] = []
            for patch_W, color in zip(patch_Wlist, view_colors):
                # Project the full patch the first time, to initialize a vertex
                # list with enough space for any possible convex hull of this
                # vertex set.
                patch_V = self._project_patch(patch_W)
                body_fill = self.ax.fill(
                    patch_V[0, :], patch_V[1, :], zorder=0,
                    edgecolor='k', facecolor=color, closed=True)[0]
                self._body_fill_dict[full_name].append(body_fill)
                # Then update the vertices for a more accurate initial draw.
                self._update_body_fill_verts(body_fill, patch_V)

    def _build_body_patches(self, use_random_colors,
                            substitute_collocated_mesh_files):
        """
        Generates body patches. self._patch_Blist stores a list of patches for
        each body (starting at body id 1). A body patch is a list of all 3D
        vertices of a piece of visual geometry.
        """
        self._patch_Blist = {}
        self._patch_Blist_colors = {}

        memq_lcm = DrakeLcm("memq://")
        memq_lcm_subscriber = Subscriber(lcm=memq_lcm,
                                         channel="DRAKE_VIEWER_LOAD_ROBOT",
                                         lcm_type=lcmt_viewer_load_robot)
        # TODO(SeanCurtis-TRI): Use SceneGraph inspection instead of mocking
        # LCM and inspecting the generated message.
        DispatchLoadMessage(self._scene_graph, memq_lcm)
        memq_lcm.HandleSubscriptions(0)
        assert memq_lcm_subscriber.count > 0
        load_robot_msg = memq_lcm_subscriber.message

        # Spawn a random color generator, in case we need to pick random colors
        # for some bodies. Each body will be given a unique color when using
        # this random generator, with each visual element of the body colored
        # the same.
        color = iter(plt.cm.rainbow(np.linspace(0, 1,
                                                load_robot_msg.num_links)))

        for i in range(load_robot_msg.num_links):
            link = load_robot_msg.link[i]

            this_body_patches = []
            this_body_colors = []
            this_color = next(color)

            for j in range(link.num_geom):
                geom = link.geom[j]
                # MultibodyPlant currently sets alpha=0 to make collision
                # geometry "invisible".  Ignore those geometries here.
                if geom.color[3] == 0:
                    continue

                X_BG = RigidTransform(
                    RotationMatrix(Quaternion(geom.quaternion)),
                    geom.position)

                if geom.type == geom.BOX:
                    assert geom.num_float_data == 3

                    # Draw a bounding box.
                    patch_G = np.vstack((
                        geom.float_data[0]/2.*np.array(
                            [-1, -1, 1, 1, -1, -1, 1, 1]),
                        geom.float_data[1]/2.*np.array(
                            [-1, 1, -1, 1, -1, 1, -1, 1]),
                        geom.float_data[2]/2.*np.array(
                            [-1, -1, -1, -1, 1, 1, 1, 1])))

                elif geom.type == geom.SPHERE:
                    assert geom.num_float_data == 1
                    radius = geom.float_data[0]
                    lati, longi = np.meshgrid(np.arange(0., 2.*math.pi, 0.5),
                                              np.arange(0., 2.*math.pi, 0.5))
                    lati = lati.ravel()
                    longi = longi.ravel()
                    patch_G = np.vstack([
                        np.sin(lati)*np.cos(longi),
                        np.sin(lati)*np.sin(longi),
                        np.cos(lati)])
                    patch_G *= radius

                elif geom.type == geom.CYLINDER:
                    assert geom.num_float_data == 2
                    radius = geom.float_data[0]
                    length = geom.float_data[1]

                    # In the lcm geometry, cylinders are along +z
                    # https://github.com/RobotLocomotion/drake/blob/last_sha_with_original_matlab/drake/matlab/systems/plants/RigidBodyCylinder.m
                    # Two circles: one at bottom, one at top.
                    sample_pts = np.arange(0., 2.*math.pi, 0.25)
                    patch_G = np.hstack(
                        [np.array([
                            [radius*math.cos(pt),
                             radius*math.sin(pt),
                             -length/2.],
                            [radius*math.cos(pt),
                             radius*math.sin(pt),
                             length/2.]]).T
                         for pt in sample_pts])

                elif geom.type == geom.MESH:
                    filename = geom.string_data
                    base, ext = os.path.splitext(filename)
                    if (ext.lower() != ".obj"
                            and substitute_collocated_mesh_files):
                        # Check for a co-located .obj file (case insensitive).
                        for f in glob.glob(base + '.*'):
                            if f[-4:].lower() == '.obj':
                                filename = f
                                break
                        if filename[-4:].lower() != '.obj':
                            raise RuntimeError(
                                "The given file " + filename + " is not "
                                "supported and no alternate " + base +
                                ".obj could be found.")
                    if not os.path.exists(filename):
                        raise FileNotFoundError(errno.ENOENT, os.strerror(
                            errno.ENOENT), filename)
                    mesh = ReadObjToSurfaceMesh(filename)
                    patch_G = np.vstack([v.r_MV() for v in mesh.vertices()]).T

                else:
                    print("UNSUPPORTED GEOMETRY TYPE {} IGNORED".format(
                        geom.type))
                    continue

                # Compute pose in body.
                patch_B = X_BG @ patch_G

                # Close path if not closed.
                if (patch_B[:, -1] != patch_B[:, 0]).any():
                    patch_B = np.hstack((patch_B, patch_B[:, 0][np.newaxis].T))

                this_body_patches.append(patch_B)
                if use_random_colors:
                    this_body_colors.append(this_color)
                else:
                    this_body_colors.append(geom.color)

            self._patch_Blist[link.name] = this_body_patches
            self._patch_Blist_colors[link.name] = this_body_colors

    def _get_view_patches(self, full_name, X_WB):
        """
        Pulls out the view patch verts for the given body index after applying
        the appropriate transform, X_WB. X_WB needs to be a RigidTransform.
        """
        patch_Wlist = []
        for patch_B in self._patch_Blist[full_name]:
            patch_W = X_WB @ patch_B
            # Add homogeneous row.
            patch_W = np.vstack((patch_W, np.ones((1, patch_W.shape[1]))))
            patch_Wlist.append(patch_W)

        colors = self._patch_Blist_colors[full_name]
        return (patch_Wlist, colors)

    def _project_patch(self, patch_W):
        """
        Project the object vertices from 3d in world frame W to 2d in view
        frame V.
        """
        patch_V = self._T_VW @ patch_W
        # Applies normalization in the perspective transformation
        # to make each projected point have z = 1. If the bottom row
        # of T_VW is [0, 0, 0, 1], this will result in an
        # orthographic projection.
        patch_V[0, :] /= patch_V[2, :]
        patch_V[1, :] /= patch_V[2, :]
        # Cut patch_V down to 2xN.
        patch_V = patch_V[:2, :]
        return patch_V

    def _update_body_fill_verts(self, body_fill, patch_V):
        """
        Takes a convex hull if necessary and uses in-place replacement of
        vertices to update the fill.
        """

        # Take a convex hull to get an accurate shape for drawing, with verts
        # coming out in ccw order.
        if patch_V.shape[1] > 3:
            hull = spatial.ConvexHull(patch_V.T)
            patch_V = np.vstack([patch_V[:, v] for v in hull.vertices]).T

        # Update the verts, padding out to the appropriate full # of verts by
        # replicating the final vertex.
        n_verts = body_fill.get_path().vertices.shape[0]
        patch_V = np.pad(
            patch_V, ((0, 0), (0, n_verts - patch_V.shape[1])), mode="edge")
        body_fill.get_path().vertices[:, :] = patch_V.T

    def draw(self, context):
        """Overrides base with the implementation."""

        pose_bundle = self._pose_bundle_port.Eval(context)
        view_dir = np.cross(self._T_VW[0, :3], self._T_VW[1, :3])
        for frame_i in range(pose_bundle.get_num_poses()):
            # SceneGraph currently sets the name in PoseBundle as
            #    "get_source_name::frame_name".
            full_name = pose_bundle.get_name(frame_i)
            model_id = pose_bundle.get_model_instance_id(frame_i)

            X_WB = RigidTransform(pose_bundle.get_pose(frame_i))
            patch_Wlist, _ = self._get_view_patches(full_name, X_WB)
            for i, patch_W in enumerate(patch_Wlist):
                # Project the object vertices from 3d in world frame W to 2d in
                # view frame V (keeps homogeneous portion, removing it later).
                patch_V = self._project_patch(patch_W)
                body_fill = self._body_fill_dict[full_name][i]
                # Use the latest vertices to update the body_fill.
                self._update_body_fill_verts(body_fill, patch_V)
                body_fill.zorder = X_WB.translation() @ view_dir
        self.ax.set_title('t = {:.1f}'.format(context.get_time()))


def ConnectPlanarSceneGraphVisualizer(builder,
                                      scene_graph,
                                      output_port=None,
                                      **kwargs):
    """Creates an instance of PlanarSceneGraphVisualizer, adds it to the
    diagram, and wires the scene_graph pose bundle output port to the input
    port of the visualizer.  Provides a comparable interface to
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

        Additional kwargs are passed through to the PlanarSceneGraphVisualizer
        constructor.

    Returns:
        The newly created PlanarSceneGraphVisualizer object.
    """
    if output_port is None:
        output_port = scene_graph.get_pose_bundle_output_port()
    visualizer = builder.AddSystem(
        PlanarSceneGraphVisualizer(scene_graph, **kwargs))
    builder.Connect(output_port, visualizer.get_input_port(0))
    return visualizer
