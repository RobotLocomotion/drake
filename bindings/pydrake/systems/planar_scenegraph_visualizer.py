import math
import warnings

import errno
import glob
import matplotlib
import matplotlib.cm as plt_cm
import numpy as np
import os
from pathlib import Path

from pydrake.common.deprecation import _warn_deprecated
from pydrake.common.value import Value
from pydrake.geometry import (
    Box,
    Convex,
    Cylinder,
    HalfSpace,
    Mesh,
    QueryObject,
    Rgba,
    Role,
    Sphere,
    optimization,
)
from pydrake.multibody.all import ContactResults
from pydrake.math import RigidTransform
from pydrake.systems.pyplot_visualizer import PyPlotVisualizer


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

    Will render contact if constructor `contact` flag is set to true and the
    corresponding contact port is connected.
    """

    def __init__(self,
                 scene_graph,
                 contact=False,
                 draw_period=None,
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
            contact: boolean flag for contact visualization, default set to
                False
            draw_period: The rate at which this class publishes to the
                visualizer.  When None, a suitable default will be used.
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

        self._geometry_query_input_port = self.DeclareAbstractInputPort(
            "geometry_query", Value(QueryObject()))
        self._contact_results_input_port = self.DeclareAbstractInputPort(
            "contact_results", Value(ContactResults()))

        self.ax.axis('equal')
        self.ax.axis('off')

        # Achieve the desired view limits.
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        default_size = self.fig.get_size_inches()
        self.fig.set_size_inches(figsize[0], figsize[1])

        # Populate body patches.
        self._build_body_patches(use_random_colors,
                                 substitute_collocated_mesh_files,
                                 scene_graph.model_inspector())

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

        self._contact_flag = contact
        # create matplotlib plots to visualise contacts and contact forces only
        # if contact flag is provided
        if self._contact_flag:
            self._contacts = self.ax.plot([], [], 'bo', ms=5)[0]
            self._contact_forces = []
            for i in range(10):
                self._contact_forces.append(
                        self.ax.plot([],
                                     [],
                                     'g-',
                                     ms=5)[0])

    def get_geometry_query_input_port(self):
        return self._geometry_query_input_port

    def get_contact_results_input_port(self):
        return self._contact_results_input_port

    @staticmethod
    def frame_name(frame_id, inspector):
        """Produces a visualizer name for a frame."""
        # The frame names have been selected to match the names produced by
        # DrakeVisualizer. That is *not* a requirement, and this visualizer can
        # choose a different naming scheme.
        if frame_id == inspector.world_frame_id():
            return "world"
        return "{}::{}".format(
            inspector.GetOwningSourceName(frame_id),
            inspector.GetName(frame_id))

    def _build_body_patches(self, use_random_colors,
                            substitute_collocated_mesh_files, inspector):
        """
        Generates body patches. self._patch_Blist stores a list of patches for
        each body (starting at body id 1). A body patch is a list of all 3D
        vertices of a piece of visual geometry.
        """
        self._patch_Blist = {}
        self._patch_Blist_colors = {}

        for frame_id in inspector.GetAllFrameIds():
            count = inspector.NumGeometriesForFrameWithRole(frame_id,
                                                            Role.kIllustration)
            if count == 0:
                continue

            frame_name = self.frame_name(frame_id, inspector)

            this_body_patches = []
            this_body_colors = []

            for g_id in inspector.GetGeometries(frame_id, Role.kIllustration):
                X_BG = inspector.GetPoseInFrame(g_id)
                shape = inspector.GetShape(g_id)

                if isinstance(shape, Box):
                    # Draw a bounding box.
                    patch_G = np.vstack((
                        shape.width()/2.*np.array(
                            [-1, -1, 1, 1, -1, -1, 1, 1]),
                        shape.depth()/2.*np.array(
                            [-1, 1, -1, 1, -1, 1, -1, 1]),
                        shape.height()/2.*np.array(
                            [-1, -1, -1, -1, 1, 1, 1, 1])))

                elif isinstance(shape, Sphere):
                    # Sphere is the only shape that allows a zero-measure. A
                    # zero-radius sphere is a point, and we skip it.
                    if shape.radius() == 0:
                        continue

                    lati, longi = np.meshgrid(np.arange(0., 2.*math.pi, 0.5),
                                              np.arange(0., 2.*math.pi, 0.5))
                    lati = lati.ravel()
                    longi = longi.ravel()
                    patch_G = np.vstack([
                        np.sin(lati)*np.cos(longi),
                        np.sin(lati)*np.sin(longi),
                        np.cos(lati)])
                    patch_G *= shape.radius()

                elif isinstance(shape, Cylinder):
                    radius = shape.radius()
                    length = shape.length()

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

                elif isinstance(shape, (Mesh, Convex)):
                    # Legacy behavior for looking for a .obj when the extension
                    # is not recognized.
                    filename = shape.filename()
                    base, ext = os.path.splitext(filename)
                    if (ext.lower() not in [".obj", ".vtk", ".gltf"]
                            and substitute_collocated_mesh_files):
                        # Check for a co-located .obj file (case insensitive).
                        for f in glob.glob(base + '.*'):
                            if f[-4:].lower() == '.obj':
                                filename = f
                                break
                        if filename[-4:].lower() != '.obj':
                            raise RuntimeError(
                                f"The given file {filename} is not "
                                f"supported and no alternate {base}"
                                ".obj could be found.")
                    if not os.path.exists(filename):
                        raise FileNotFoundError(errno.ENOENT, os.strerror(
                            errno.ENOENT), filename)
                    temp_mesh = Mesh(filename, shape.scale())
                    # TODO(21125): When Convex shape *always* has a
                    # convex hull available, use it instead of recomputing it
                    # here.
                    convex_hull = _MakeConvexHull(temp_mesh)
                    patch_G = np.empty((3, convex_hull.num_vertices()))
                    for i in range(convex_hull.num_vertices()):
                        patch_G[:, i] = convex_hull.vertex(i)

                elif isinstance(shape, HalfSpace):
                    # For a half space, we'll simply create a large box with
                    # the top face at z = 0, the bottom face at z = -1 and the
                    # far corners at +/- 50 in the x- and y- directions.
                    x = 50
                    y = 50
                    z = -1
                    patch_G = np.vstack((
                        x * np.array([-1, -1,  1,  1, -1, -1,  1, 1]),
                        y * np.array([-1,  1, -1,  1, -1,  1, -1, 1]),
                        z * np.array([-1, -1, -1, -1,  0,  0,  0, 0])))

                # TODO(SeanCurtis-TRI): Provide support for capsule and
                # ellipsoid.
                else:
                    print("UNSUPPORTED GEOMETRY TYPE {} IGNORED".format(
                        type(shape)))
                    continue

                # Compute pose in body.
                patch_B = X_BG @ patch_G

                # Close path if not closed.
                if (patch_B[:, -1] != patch_B[:, 0]).any():
                    patch_B = np.hstack((patch_B, patch_B[:, 0][np.newaxis].T))

                this_body_patches.append(patch_B)
                if not use_random_colors:
                    # If we need to use random colors, we apply them after the
                    # fact. See below.
                    props = inspector.GetIllustrationProperties(g_id)
                    assert props is not None
                    rgba = props.GetPropertyOrDefault(
                        "phong", "diffuse",  Rgba(0.9, 0.9, 0.9, 1.0))
                    color = np.array((rgba.r(), rgba.g(), rgba.b(), rgba.a()))
                    this_body_colors.append(color)

            self._patch_Blist[frame_name] = this_body_patches
            self._patch_Blist_colors[frame_name] = this_body_colors

        # Spawn a random color generator. Each body will be given a unique
        # color when using this random generator, with each visual element of
        # the body colored the same.
        if use_random_colors:
            color = iter(plt_cm.rainbow(
                np.linspace(0, 1, len(self._patch_Blist_colors))))
            for name in self._patch_Blist_colors.keys():
                this_color = next(color)
                patch_count = len(self._patch_Blist[name])
                self._patch_Blist_colors[name] = [this_color] * patch_count

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

    def _contact_force_projection(self, contact_points, contact_forces):
        """
        Internal function that projects 3D force information, contact and force
        magnitude onto the 2D visualization plane
        """

        # visualising contact points
        contact_aug = np.vstack((contact_points, np.ones((
            1, contact_points.shape[1]))))
        point_2d = self._T_VW @ contact_aug
        point_2d[0, :] /= point_2d[2, :]
        point_2d[1, :] /= point_2d[2, :]
        point_2d = point_2d[:2, :]

        # visualising contact forces
        contact_force_vis_points = contact_points + contact_forces
        contact_force_vis_aug = np.vstack((contact_force_vis_points, np.ones
                                           ((1,
                                             contact_force_vis_points.shape[1]
                                             ))))
        contact_vis_2d = self._T_VW @ contact_force_vis_aug

        contact_vis_2d[0, :] /= contact_vis_2d[2, :]
        contact_vis_2d[1, :] /= contact_vis_2d[2, :]
        contact_vis_2d = contact_vis_2d[:2, :]

        return point_2d, contact_vis_2d


    def _update_body_fill_verts(self, body_fill, patch_V):
        """
        Takes a convex hull if necessary and uses in-place replacement of
        vertices to update the fill.
        """

        # Take a convex hull to get an accurate shape for drawing, with verts
        # coming out in ccw order.
        if patch_V.shape[1] > 3:
            vpoly = optimization.VPolytope(patch_V)
            patch_V = vpoly.GetMinimalRepresentation().vertices()

        # Update the verts, padding out to the appropriate full # of verts by
        # replicating the final vertex.
        n_verts = body_fill.get_path().vertices.shape[0]
        patch_V = np.pad(
            patch_V, ((0, 0), (0, n_verts - patch_V.shape[1])), mode="edge")
        body_fill.get_path().vertices[:, :] = patch_V.T

    def draw(self, context):
        """Overrides base with the implementation."""
        query_object = self._geometry_query_input_port.Eval(context)
        if self._contact_flag:
            contact_results = self._contact_results_input_port.Eval(context)
        inspector = query_object.inspector()

        view_dir = np.cross(self._T_VW[0, :3], self._T_VW[1, :3])
        for frame_id in inspector.GetAllFrameIds():
            frame_name = self.frame_name(frame_id, inspector)
            if frame_name not in self._patch_Blist:
                continue
            X_WB = query_object.GetPoseInWorld(frame_id)

            patch_Wlist, _ = self._get_view_patches(frame_name, X_WB)
            for i, patch_W in enumerate(patch_Wlist):
                # Project the object vertices from 3d in world frame W to 2d in
                # view frame V (keeps homogeneous portion, removing it later).
                patch_V = self._project_patch(patch_W)
                body_fill = self._body_fill_dict[frame_name][i]
                # Use the latest vertices to update the body_fill.
                self._update_body_fill_verts(body_fill, patch_V)
                body_fill.zorder = X_WB.translation() @ view_dir
        self.ax.set_title('t = {:.1f}'.format(context.get_time()))

        # contact visualisation
        # draws the corresponding contact information onto plt figure
        if self._contact_flag:
            points = []
            forces = []
            # extract contact information
            for id in range(contact_results.num_point_pair_contacts()):
                point_contact_info = contact_results.point_pair_contact_info(id)
                points.append(point_contact_info.contact_point())
                forces.append(point_contact_info.contact_force())
            for id in range(contact_results.num_hydroelastic_contacts()):
                contact_info = contact_results.hydroelastic_contact_info(id)
                surface_data = contact_info.contact_surface()
                # extracts only translational force
                spatial_force = contact_info.F_Ac_W().translational()
                points.append(surface_data.centroid())
                forces.append(spatial_force)

            if len(points) == 0:
                return
            if len(forces) == 0:
                print("no contact forces")
                return
            # experimental scaling down of forces for visualisation
            # TODO: (@kamiradi) forces are randomly scaled down to visualise
            # them on the matplotlib figure, maybe another way to do this?

            # project 3D to 2D
            contact_plt, contact_force = self._contact_force_projection(
                np.asarray(points).transpose(),
                np.asarray(forces).transpose()*0.05
            )

            # update contact points data for visualization
            self._contacts.set_xdata(contact_plt[0, :])
            self._contacts.set_ydata(contact_plt[1, :])

            # hard coded only 10 contact force lines
            if contact_plt.shape[1] >= 10:
                contact_plt = contact_plt[:, :10]
                contact_force = contact_force[:, :10]

            # update contact force information for visualization
            for i in range(contact_force.shape[1]):
                self._contact_forces[i].set_data(
                    [contact_plt[0, i], contact_force[0, i]],
                    [contact_plt[1, i], contact_force[1, i]])


def ConnectPlanarSceneGraphVisualizer(builder,
                                      scene_graph,
                                      output_port=None,
                                      contact_port=None,
                                      **kwargs):
    """Creates an instance of PlanarSceneGraphVisualizer, adds it to the
    diagram, and wires the scene_graph pose bundle output port to the input
    port of the visualizer. Optionally, contact_port - on provision - is wired
    to the visualizer. Provides an interface comparable to
    DrakeVisualizer.AddToBuilder.

    Args:
        builder: The diagram builder used to construct the Diagram.
        scene_graph: The SceneGraph in builder containing the geometry to be
            visualized.
        output_port: (optional) If not None, then output_port will be connected
            to the visualizer input port instead of the scene_graph.
            get_query_output_port().  This is required, for instance,
            when the SceneGraph is inside a Diagram, and we must connect the
            exposed port to the visualizer instead of the original SceneGraph
            port.
        contact_port: (optional) If not None, then contact_port will be
            connected to the visualizer's input port. If not provided, contacts
            are not visualized.

        Additional kwargs are passed through to the PlanarSceneGraphVisualizer
        constructor.

    Returns:
        The newly created PlanarSceneGraphVisualizer object.
    """
    if output_port is None:
        output_port = scene_graph.get_query_output_port()

    if contact_port is None:
        visualizer = builder.AddSystem(
            PlanarSceneGraphVisualizer(
                scene_graph,
                contact=False,
                **kwargs))
        builder.Connect(
                output_port,
                visualizer.get_geometry_query_input_port())
    else:
        visualizer = builder.AddSystem(
                PlanarSceneGraphVisualizer(scene_graph,
                                           contact=True,
                                           **kwargs))
        builder.Connect(
                output_port,
                visualizer.get_geometry_query_input_port())
        builder.Connect(
                contact_port,
                visualizer.get_contact_results_input_port())

    return visualizer
