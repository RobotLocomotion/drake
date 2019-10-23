import math

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import scipy as sp
from scipy import spatial

from drake import lcmt_viewer_load_robot
from pydrake.common.eigen_geometry import Quaternion
from pydrake.geometry import DispatchLoadMessage
from pydrake.lcm import DrakeMockLcm, Subscriber
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

    TView specifies the 3x4 view projection matrix. For planar orthographic
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

    TView can be any valid view projection matrix. If the bottom row is
    [0, 0, 0, 1], the view projection will be an orthographic projection.

    xlim and ylim don't technically provide extra functionality, but I think
    it's easier to keep handle scaling with xlim and ylim and view plane
    selection and *maybe* offsetting with the projection matrix.
    """

    def __init__(self,
                 scene_graph,
                 draw_period=1./30,
                 Tview=np.array([[1., 0., 0., 0.],
                                 [0., 0., 1., 0.],
                                 [0., 0., 0., 1.]]),
                 xlim=[-1., 1],
                 ylim=[-1, 1],
                 facecolor=[1, 1, 1],
                 use_random_colors=False,
                 ax=None):
        """
        Args:
            scene_graph: A SceneGraph object.
            draw_period: The rate at which this class publishes to the
                visualizer.
            Tview: The view projection matrix.
            xlim: View limit into the scene.
            ylim: View limit into the scene.
            facecolor: Passed through to figure() and sets background color.
                Both color name strings and RGB triplets are allowed. Defaults
                to white.
            use_random_colors: If set to True, will render each body with a
                different color. (Multiple visual elements on the same body
                will be the same color.)
            ax: If supplied, the visualizer will draw onto those axes instead
                of creating a new set of axes. The visualizer will still change
                the view range and figure size of those axes.
        """
        default_size = matplotlib.rcParams['figure.figsize']
        scalefactor = (ylim[1]-ylim[0])/(xlim[1]-xlim[0])
        figsize = (default_size[0], default_size[0]*scalefactor)

        PyPlotVisualizer.__init__(self, facecolor=facecolor, figsize=figsize,
                                  ax=ax, draw_timestep=draw_period)
        self.set_name('planar_multibody_visualizer')

        self._scene_graph = scene_graph
        self._Tview = Tview

        # Pose bundle (from SceneGraph) input port.
        self.DeclareAbstractInputPort("lcm_visualization",
                                      AbstractValue.Make(PoseBundle(0)))

        self.ax.axis('equal')
        self.ax.axis('off')

        # Achieve the desired view limits.
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        default_size = self.fig.get_size_inches()
        scalefactor = (ylim[1]-ylim[0])/(xlim[1]-xlim[0])
        self.fig.set_size_inches(default_size[0],
                                 default_size[0]*scalefactor)

        # Populate body patches.
        self._build_view_patches(use_random_colors)

        # Populate the body fill list -- which requires doing most of a draw
        # pass, but with an ax.fill() command rather than an in-place
        # replacement of vertex positions to initialize the draw patches. The
        # body fill list stores the ax patch objects in the order they were
        # spawned (i.e. by body, and then by order of view_patches). Drawing
        # the tree should update them by iterating over bodies and patches in
        # the same order.
        self._body_fill_dict = {}
        n_bodies = len(self._view_patches.keys())
        tf = np.eye(4)
        for full_name in self._view_patches.keys():
            view_patches, view_colors = self._get_view_patches(full_name, tf)
            self._body_fill_dict[full_name] = []
            for patch, color in zip(view_patches, view_colors):
                # Project the full patch the first time, to initialize a vertex
                # list with enough space for any possible convex hull of this
                # vertex set.
                patch_proj = np.dot(self._Tview, patch)
                self._body_fill_dict[full_name] += self.ax.fill(
                    patch_proj[0, :], patch_proj[1, :], zorder=0,
                    edgecolor='k', facecolor=color, closed=True)

    def _build_view_patches(self, use_random_colors):
        """
        Generates view patches. self._view_patches stores a list of
        view patches for each body (starting at body id 1). A view patch is a
        list of all 3D vertices of a piece of visual geometry.
        """
        self._view_patches = {}
        self._view_patch_colors = {}

        mock_lcm = DrakeMockLcm()
        mock_lcm_subscriber = Subscriber(lcm=mock_lcm,
                                         channel="DRAKE_VIEWER_LOAD_ROBOT",
                                         lcm_type=lcmt_viewer_load_robot)
        DispatchLoadMessage(self._scene_graph, mock_lcm)
        mock_lcm.HandleSubscriptions(0)
        assert mock_lcm_subscriber.count > 0
        load_robot_msg = mock_lcm_subscriber.message

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

                element_local_tf = RigidTransform(
                    RotationMatrix(Quaternion(geom.quaternion)),
                    geom.position)

                if geom.type == geom.BOX:
                    assert geom.num_float_data == 3

                    # Draw a bounding box.
                    patch = np.vstack((
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
                    patch = np.vstack([
                        np.sin(lati)*np.cos(longi),
                        np.sin(lati)*np.sin(longi),
                        np.cos(lati)])
                    patch *= radius

                elif geom.type == geom.CYLINDER:
                    assert geom.num_float_data == 2
                    radius = geom.float_data[0]
                    length = geom.float_data[1]

                    # In the lcm geometry, cylinders are along +z
                    # https://github.com/RobotLocomotion/drake/blob/last_sha_with_original_matlab/drake/matlab/systems/plants/RigidBodyCylinder.m
                    # Two circles: one at bottom, one at top.
                    sample_pts = np.arange(0., 2.*math.pi, 0.25)
                    patch = np.hstack(
                        [np.array([
                            [radius*math.cos(pt),
                             radius*math.sin(pt),
                             -length/2.],
                            [radius*math.cos(pt),
                             radius*math.sin(pt),
                             length/2.]]).T
                         for pt in sample_pts])

                else:
                    print("UNSUPPORTED GEOMETRY TYPE {} IGNORED".format(
                        geom.type))
                    continue

                patch = np.vstack((patch, np.ones((1, patch.shape[1]))))
                patch = np.dot(element_local_tf.GetAsMatrix4(), patch)

                # Close path if not closed.
                if (patch[:, -1] != patch[:, 0]).any():
                    patch = np.hstack((patch, patch[:, 0][np.newaxis].T))

                this_body_patches.append(patch)
                if use_random_colors:
                    this_body_colors.append(this_color)
                else:
                    this_body_colors.append(geom.color)

            self._view_patches[link.name] = this_body_patches
            self._view_patch_colors[link.name] = this_body_colors

    def _get_view_patches(self, full_name, tf):
        """
        Pulls out the view patch verts for the given body index after applying
        the appropriate TF.
        """
        transformed_patches = [np.dot(tf, patch)
                               for patch in self._view_patches[full_name]]
        colors = self._view_patch_colors[full_name]
        return (transformed_patches, colors)

    def draw(self, context):
        """
        Evaluates the robot state and draws it. Can be passed either a raw
        state vector, or an input context.
        """
        pose_bundle = self.EvalAbstractInput(context, 0).get_value()
        view_dir = np.cross(self._Tview[0, :3], self._Tview[1, :3])
        for frame_i in range(pose_bundle.get_num_poses()):
            # SceneGraph currently sets the name in PoseBundle as
            #    "get_source_name::frame_name".
            full_name = pose_bundle.get_name(frame_i)
            model_id = pose_bundle.get_model_instance_id(frame_i)

            pose = pose_bundle.get_pose(frame_i).matrix()
            view_patches, _ = self._get_view_patches(full_name, pose)
            for i, patch in enumerate(view_patches):
                # Project the object vertices to 2d.
                patch_proj = np.dot(self._Tview, patch)
                # Applies normalization in the perspective transformation
                # to make each projected point have z = 1. If the bottom row
                # of Tview is [0, 0, 0, 1], this will result in an
                # orthographic projection.
                patch_proj[0, :] /= patch_proj[2, :]
                patch_proj[1, :] /= patch_proj[2, :]
                # Cut patch_proj down to 2xN.
                patch_proj = patch_proj[:2, :]
                # Take a convex hull to get an accurate shape for drawing,
                # with verts coming out in ccw order.
                if patch_proj.shape[1] > 3:
                    hull = spatial.ConvexHull(np.transpose(patch_proj))
                    patch_proj = np.transpose(
                        np.vstack([patch_proj[:, v] for v in hull.vertices]))
                n_verts = self._body_fill_dict[full_name][i].get_path().\
                    vertices.shape[0]
                # Update the verts, padding out to the appropriate full # of
                # verts by replicating the final vertex.
                patch_proj = np.pad(
                    patch_proj, ((0, 0), (0, n_verts - patch_proj.shape[1])),
                    mode="edge")
                self._body_fill_dict[full_name][i].get_path().vertices[:, :] = np.transpose(patch_proj)  # noqa
                self._body_fill_dict[full_name][i].zorder = np.dot(
                    pose[:3, 3], view_dir)
        self.ax.set_title('t = {:.1f}'.format(context.get_time()))
