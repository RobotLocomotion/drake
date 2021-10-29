"""
Provides utilities for communicating with the browser-based visualization
package, Meshcat:
      https://github.com/rdeits/meshcat
"""
import argparse
import os
import uuid
import warnings
import webbrowser

import numpy as np

from pydrake.common.deprecation import _warn_deprecated
from pydrake.common.value import AbstractValue
from pydrake.geometry import (
    Box, ConvertVolumeToSurfaceMesh, Convex, Cylinder, Mesh, Sphere,
    FrameId, QueryObject, Role, SceneGraph, VolumeMesh
)
from pydrake.lcm import DrakeLcm, Subscriber
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.framework import LeafSystem, PublishEvent, TriggerType
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

# To help avoid small simulation timesteps, we use a default period that has an
# exact representation in binary floating point; see drake#15021 for details.
_DEFAULT_PUBLISH_PERIOD = 1 / 32.


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


class StringToRoleAction(argparse.Action):
    """
    Action that converts the string 'proximity' or 'illustration' to the
    corresponding Role enumeration value.
    """
    def __init__(self, option_strings, dest, nargs=None, **kwargs):
        if nargs is not None:
            raise ValueError("nargs not allowed")
        super().__init__(option_strings, dest, **kwargs)

    def __call__(self, parser, namespace, values, option_string=None):
        assert isinstance(values, str)

        if values == 'proximity':
            mapped_value = Role.kProximity
        elif values == 'illustration':
            mapped_value = Role.kIllustration
        else:
            raise ValueError(f"Role parameter got invalid value {s}")

        setattr(namespace, self.dest, mapped_value)


class HydroTriSurface(g.Geometry):
    """
    Unique representation of the triangle surface mesh associated with
    hydroelastic mesh representations. In this case, it's important that we
    support per-face normals (we want as honest a representation of the
    object as possible).

    A mesh consisting of an arbitrary collection of triangular faces. To
    construct one, you need to pass in a collection of vertices as an 3Nx3
    array and a collection of normals as a 3Nx3 array as well. The triangles
    are implied by every triple of vertex/normals. This does *not* allow
    re-use of vertices because it encodes a classic OpenGl vertex buffer
    format.

    For example, to create a square made out of two adjacent triangles, we
    could do:
    vertices = np.array([
        [0, 0, 0],  # the first vertex is at [0, 0, 0]
        [1, 0, 0],
        [1, 0, 1],
        [0, 0, 1],
        [0, 0, 0],
        [1, 0, 1]
    ])
    normals = np.array([
        [0, 1, 0],  # The square is planar, all normals point in the same dir.
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
    ])
    mesh = HydroTriSurface(vertices, normals)
    """
    __slots__ = ["vertices", "normals"]

    def __init__(self, vertices, normals):
        super(HydroTriSurface, self).__init__()

        vertices = np.asarray(vertices, dtype=np.float32)
        normals = np.asarray(normals, dtype=np.float32)
        assert (vertices.shape[0] % 3) == 0, "vertices must have 3N values"
        assert vertices.shape[1] == 3, "`vertices` must be an 3Nx3 array"
        assert normals.shape[1] == 3, "`normals` must be an 3Nx3 array"
        assert vertices.shape[0] == normals.shape[0], \
               "'vertices' and 'normals' must be the same size"
        self.vertices = vertices
        self.normals = normals

    def lower(self, object_data):
        attrs = {u"position": g.pack_numpy_array(self.vertices.T),
                 u"normal": g.pack_numpy_array(self.normals.T)}
        return {
            u"uuid": self.uuid,
            u"type": u"BufferGeometry",
            u"data": {
                u"attributes": attrs
            }
        }


class MeshcatVisualizer(LeafSystem):
    """
    MeshcatVisualizer is a System block that connects to the query output port
    of a SceneGraph and visualizes the scene in Meshcat.

    The most common workflow would be to run::

        bazel run @meshcat_python//:meshcat-server

    in another terminal, open the url printed in that terminal in your browser,
    then to run drake apps (potentially many times) that publish to that
    default url.
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

        parser.add_argument(
            "--meshcat_role", action=StringToRoleAction,
            default=Role.kIllustration, choices=['illustration', 'proximity'],
            help="Defines the role of the geometry to visualize")

        parser.add_argument(
            "--meshcat_hydroelastic", action="store_true", default=False,
            help="If --role=proximity, then any geometry with a hydroelastic "
                 "mesh representation will be rendered as that discrete mesh "
                 "instead of the declared primitive.")

    def __init__(self,
                 scene_graph=None,
                 draw_period=_DEFAULT_PUBLISH_PERIOD,
                 prefix="drake",
                 zmq_url="default",
                 open_browser=None,
                 frames_to_draw=[],
                 frames_opacity=1.,
                 axis_length=0.15,
                 axis_radius=0.006,
                 delete_prefix_on_load=True,
                 role=Role.kIllustration,
                 prefer_hydro=False,
                 **kwargs):
        """
        Args:
            scene_graph: A SceneGraph object.  This argument is optional, and
                is only needed to enable calls to ``load()`` without providing
                a Context.
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
            frames_to_draw: a list or array containing pydrake.geometry.FrameId
                for which body frames to draw. Note that these are frames
                registered within the scene_graph. For multibody frames, users
                may obtain the ids from plant using GetBodyFrameIdIfExists.
                Currently, frames that are not body frames are not supported
                as they do not exist in the scene_graph yet.
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
            role: Renders geometry of the specified pydrake.geometry.Role
                type -- defaults to Role.kIllustration to draw visual geometry,
                and also supports Role.kProximity to draw collision geometry.
            prefer_hydro: If True (and role == Role.kProximity) any geometry
                that has a mesh hydroelastic representation will be visualized
                by that discrete mesh and not the declared primitive. In the
                case of *compliant* hydroelastic geometries, only the surface
                of the volume mesh will be drawn. This is *not* the *contact*
                surface used to characterize contact between two geometries
                with hydroelastic representations -- it is the mesh
                representations of those geometries.

        Additional kwargs will be passed to the meshcat.Visualizer constructor.
        Note:
            This call will not return until it connects to the
            ``meshcat-server``.
        """
        LeafSystem.__init__(self)

        self.set_name('meshcat_visualizer')
        self.DeclarePeriodicPublish(draw_period, 0.0)
        self.draw_period = draw_period
        self._delete_prefix_on_load = delete_prefix_on_load
        if role not in [Role.kIllustration, Role.kProximity]:
            raise ValueError("Unsupported role type specified: ", role)
        self._role = role
        self._prefer_hydro = prefer_hydro

        # Recording.
        self._is_recording = False
        self.reset_recording()

        self._scene_graph = scene_graph

        self._geometry_query_input_port = self.DeclareAbstractInputPort(
            "geometry_query", AbstractValue.Make(QueryObject()))

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

        # Set background color (to match drake-visualizer).
        self.vis['/Background'].set_property("top_color", [0.95, 0.95, 1.0])
        self.vis['/Background'].set_property("bottom_color", [.32, .32, .35])

        if open_browser:
            webbrowser.open(self.vis.url())

        def on_initialize(context, event):
            self.load(context)

        # TODO(russt): #13776 recommends we stop using initialization events
        # for this.
        self.DeclareInitializationEvent(
            event=PublishEvent(
                trigger_type=TriggerType.kInitialization,
                callback=on_initialize))

        # TODO(russt): Move this to a cache entry as in DrakeVisualizer.
        # Requires #14287.
        self._dynamic_frames = []

        # drawing coordinate frames
        self.frames_to_draw = set(frames_to_draw)
        self.frames_opacity = frames_opacity
        self.axis_length = axis_length
        self.axis_radius = axis_radius

    def get_geometry_query_input_port(self):
        return self._geometry_query_input_port

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

    def delete_prefix(self):
        """
        Manually delete the meshcat prefix path specified in the constructor.
        All objects/transforms at or below this path will be removed.  This
        effectively clears all previous visualizations.  You will need to call
        ``load()`` directly, or trigger the initialization event that occurs at
        the beginning of a simulation, to see any visualizations.
        """
        self.vis[self.prefix].delete()

    def load(self, context=None):
        """
        Loads ``meshcat`` visualization elements.

        Precondition:
            Either the context is a valid Context for this system with the
            geometry_query port connected or the ``scene_graph`` passed in the
            constructor must be a valid SceneGraph.
        """
        if self._delete_prefix_on_load:
            self.vis[self.prefix].delete()

        if context and self.get_geometry_query_input_port().HasValue(context):
            inspector = self.get_geometry_query_input_port().Eval(
                context).inspector()
        elif self._scene_graph:
            inspector = self._scene_graph.model_inspector()
        else:
            raise RuntimeError(
                "You must provide a valid Context for this system with the "
                "geometry_query port connected or the ``scene_graph`` passed "
                "in the constructor must be a valid SceneGraph.")

        vis = self.vis[self.prefix]
        # Make a fixed-seed generator for random colors for bodies.
        color_generator = np.random.RandomState(seed=42)
        for frame_id in inspector.GetAllFrameIds():
            count = inspector.NumGeometriesForFrameWithRole(
                frame_id, self._role)
            if count == 0:
                continue
            if frame_id == inspector.world_frame_id():
                name = "world"
            else:
                # Note: MBP declares frames with SceneGraph using `::`, we
                # replace those with `/` here to expose the full tree to
                # meshcat.
                name = (inspector.GetOwningSourceName(frame_id) + "/"
                        + inspector.GetName(frame_id).replace("::", "/"))

            frame_vis = vis[name]
            for g_id in inspector.GetGeometries(frame_id, self._role):
                color = 0xe5e5e5  # default color
                alpha = 1.0
                hydro_mesh = None
                if self._role == Role.kIllustration:
                    props = inspector.GetIllustrationProperties(g_id)
                    if props and props.HasProperty("phong", "diffuse"):
                        rgba = props.GetProperty("phong", "diffuse")
                        # Convert Rgba from [0-1] to hex 0xRRGGBB.
                        color = int(255*rgba.r())*256**2
                        color += int(255*rgba.g())*256
                        color += int(255*rgba.b())
                        alpha = rgba.a()
                elif self._role == Role.kProximity:
                    # Pick a random color to make collision geometry
                    # visually distinguishable.
                    color = color_generator.randint(2**(24))
                    if self._prefer_hydro:
                        hydro_mesh = inspector. \
                            maybe_get_hydroelastic_mesh(g_id)

                material = g.MeshLambertMaterial(
                    color=color, transparent=alpha != 1., opacity=alpha)

                shape = inspector.GetShape(g_id)
                X_FG = inspector.GetPoseInFrame(g_id).GetAsMatrix4()
                if hydro_mesh is not None:
                    # We've got a hydroelastic mesh to load.
                    surface_mesh = hydro_mesh
                    if isinstance(hydro_mesh, VolumeMesh):
                        surface_mesh = ConvertVolumeToSurfaceMesh(hydro_mesh)
                    v_count = len(surface_mesh.triangles()) * 3
                    vertices = np.empty((v_count, 3), dtype=float)
                    normals = np.empty((v_count, 3), dtype=float)

                    mesh_verts = surface_mesh.vertices()
                    v = 0
                    for face in surface_mesh.triangles():
                        p_MA = mesh_verts[int(face.vertex(0))]
                        p_MB = mesh_verts[int(face.vertex(1))]
                        p_MC = mesh_verts[int(face.vertex(2))]
                        vertices[v, :] = tuple(p_MA)
                        vertices[v + 1, :] = tuple(p_MB)
                        vertices[v + 2, :] = tuple(p_MC)

                        p_AB_M = p_MB - p_MA
                        p_AC_M = p_MC - p_MA
                        n_M = np.cross(p_AB_M, p_AC_M)
                        nhat_M = n_M / np.sqrt(n_M.dot(n_M))

                        normals[v, :] = nhat_M
                        normals[v + 1, :] = nhat_M
                        normals[v + 2, :] = nhat_M

                        v += 3
                    geom = HydroTriSurface(vertices, normals)
                elif isinstance(shape, Box):
                    geom = g.Box([shape.width(), shape.depth(),
                                  shape.height()])
                elif isinstance(shape, Sphere):
                    geom = g.Sphere(shape.radius())
                elif isinstance(shape, Cylinder):
                    geom = g.Cylinder(shape.length(), shape.radius())
                    # In Drake, cylinders are along +z
                    # In meshcat, cylinders are along +y

                    R_GC = RotationMatrix.MakeXRotation(np.pi/2.0).matrix()
                    X_FG[0:3, 0:3] = X_FG[0:3, 0:3].dot(R_GC)
                elif isinstance(shape, (Mesh, Convex)):
                    geom = g.ObjMeshGeometry.from_file(
                        shape.filename()[0:-3] + "obj")
                    # Attempt to find a texture for the object by looking for
                    # an identically-named *.png next to the model.
                    # TODO(gizatt): Support .MTLs and prefer them over png,
                    # since they're both more expressive and more standard.
                    # TODO(gizatt): In the long term, this kind of material
                    # information should be gleaned from the SceneGraph
                    # constituents themselves, so that we visualize what the
                    # simulation is *actually* reasoning about rather than what
                    # files happen to be present.
                    candidate_texture_path = shape.filename()[0:-3] + "png"
                    if os.path.exists(candidate_texture_path):
                        material = g.MeshLambertMaterial(
                            map=g.ImageTexture(image=g.PngImage.from_file(
                                candidate_texture_path)))
                    # Make the uuid's deterministic for mesh geometry, to
                    # support caching at the zmqserver.  This means that
                    # multiple (identical) geometries may have the same UUID,
                    # but testing suggests that meshcat + three.js are ok with
                    # it.
                    geom.uuid = str(uuid.uuid5(
                        uuid.NAMESPACE_X500, geom.contents + "mesh"))
                    material.uuid = str(uuid.uuid5(
                        uuid.NAMESPACE_X500, geom.contents
                        + "material"))
                    X_FG = X_FG.dot(tf.scale_matrix(shape.scale()))
                else:
                    warnings.warn(f"Unsupported shape {shape} ignored")
                    continue
                geometry_vis = frame_vis[str(g_id.get_value())]
                geometry_vis.set_object(geom, material)
                geometry_vis.set_transform(X_FG)

                if frame_id in self.frames_to_draw:
                    AddTriad(
                        self.vis,
                        name=name,
                        prefix=self.prefix + "/" + name,
                        length=self.axis_length,
                        radius=self.axis_radius,
                        opacity=self.frames_opacity
                    )
                    self.frames_to_draw.remove(frame_id)

            if frame_id != inspector.world_frame_id():
                self._dynamic_frames.append({
                    "id": frame_id,
                    "name": name,
                })

        # Loop through the input frames_to_draw list and warn the user if the
        # frame_id does not exist in the scene graph.
        for frame_id in self.frames_to_draw:
            warnings.warn(f"Non-existent frame {frame_id} ignored")
            continue

    def DoPublish(self, context, event):
        # TODO(SeanCurtis-TRI) We want to be able to use this visualizer to
        # draw without having it part of a Simulator. That means we'd like
        # vis.Publish(context) to work. Currently, pydrake offers no mechanism
        # to declare a forced event. However, by overriding DoPublish and
        # putting the forced event callback code in the override, we can
        # simulate it.
        # We need to bind a mechanism for declaring forced events so we don't
        # have to resort to overriding the dispatcher.

        LeafSystem.DoPublish(self, context, event)

        vis = self.vis[self.prefix]

        query_object = self.get_geometry_query_input_port().Eval(context)

        for frame in self._dynamic_frames:
            frame_vis = vis[frame['name']]
            X_WF = query_object.GetPoseInWorld(frame['id'])
            frame_vis.set_transform(X_WF.GetAsMatrix4())
            if self._is_recording:
                with self._animation.at_frame(
                        frame_vis, self._recording_frame_num) as f:
                    f.set_transform(X_WF.GetAsMatrix4())

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
    forces. It is connected to the contact results output port of
    SceneGraph's associated MultibodyPlant.
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

        # Contact results input port from MultibodyPlant
        self.DeclareAbstractInputPort(
            "contact_results", AbstractValue.Make(ContactResults()))

        # This system has undeclared states, see #4330.
        self._published_contacts = []

        # Zap any previous contact forces on this prefix
        vis = self._meshcat_viz.vis[self._meshcat_viz.prefix]["contact_forces"]
        vis.delete()

    def DoPublish(self, context, event):
        # TODO(SeanCurtis-TRI) We want to be able to use this visualizer to
        # draw without having it part of a Simulator. That means we'd like
        # vis.Publish(context) to work. Currently, pydrake offers no mechanism
        # to declare a forced event. However, by overriding DoPublish and
        # putting the forced event callback code in the override, we can
        # simulate it.
        # We need to bind a mechanism for declaring forced events so we don't
        # have to resort to overriding the dispatcher.

        LeafSystem.DoPublish(self, context, event)

        contact_results = self.EvalAbstractInput(context, 0).get_value()

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

            # The contact frame's origin is located at contact_point and is
            # oriented such that Cy is aligned with the contact force.
            p_WC = contact_info.contact_point()  # documented as in world.
            if force_norm < 1e-6:
                # We cannot rely on self._force_threshold to determine if the
                # force can be normalized; that threshold can be zero.
                R_WC = RotationMatrix()
            else:
                fhat_C_W = contact_info.contact_force() / force_norm
                R_WC = RotationMatrix.MakeFromOneVector(b_A=fhat_C_W,
                                                        axis_index=1)
            X_WC = RigidTransform(R=R_WC, p=p_WC)
            cvis.set_transform(X_WC.GetAsMatrix4())

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

    .. pydrake_system::

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
        # TODO(SeanCurtis-TRI) We want to be able to use this visualizer to
        # draw without having it part of a Simulator. That means we'd like
        # vis.Publish(context) to work. Currently, pydrake offers no mechanism
        # to declare a forced event. However, by overriding DoPublish and
        # putting the forced event callback code in the override, we can
        # simulate it.
        # We need to bind a mechanism for declaring forced events so we don't
        # have to resort to overriding the dispatcher.

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


def ConnectMeshcatVisualizer(builder, scene_graph=None, output_port=None,
                             **kwargs):
    """Creates an instance of MeshcatVisualizer, adds it to the diagram, and
    wires the scene_graph query output port to the input port of the
    visualizer.  Provides an interface comparable to
    DrakeVisualizer.AddToBuilder.

    Args:
        builder: The diagram builder used to construct the Diagram.
        scene_graph: (optional) The SceneGraph in builder containing the
            geometry to be visualized.  At least one of scene_graph or
            output_port must be valid (not None).
        output_port: (optional) If not None, then output_port will be connected
            to the visualizer input port.  This is required, for instance, when
            the SceneGraph is inside a Diagram, and we must connect the
            exported port to the visualizer instead of the original SceneGraph
            port.

        Additional kwargs are passed through to the MeshcatVisualizer
        constructor.

    Returns:
        The newly created MeshcatVisualizer object.

    Note:
        Passing a scene_graph object also makes it possible to call
        ``visualizer.load()``, without passing a ``Context`` to the
        constructed visualizer.
    """
    visualizer = builder.AddSystem(MeshcatVisualizer(scene_graph, **kwargs))

    if output_port is None:
        assert scene_graph, ("If no output_port is specified, then the "
                             "scene_graph must be valid.")
        output_port = scene_graph.get_query_output_port()
    builder.Connect(output_port, visualizer.get_geometry_query_input_port())
    return visualizer
