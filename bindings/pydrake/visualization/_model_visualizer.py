import copy
from enum import Enum
import logging
import os
from pathlib import Path
import time
from webbrowser import open as _webbrowser_open

import numpy as np

from pydrake.geometry import (
    Box,
    Cylinder,
    GeometryInstance,
    MakePhongIllustrationProperties,
    MeshcatCone,
    Role,
    Rgba,
    StartMeshcat,
)
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.meshcat import JointSliders
from pydrake.planning import RobotDiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.planar_scenegraph_visualizer import (
    ConnectPlanarSceneGraphVisualizer,
)
from pydrake.systems.sensors import (
    ApplyCameraConfig,
    CameraConfig,
)
from pydrake.visualization import (
    VisualizationConfig,
    ApplyVisualizationConfig,
)
from pydrake.visualization._triad import (
    AddFrameTriadIllustration,
)


class ModelVisualizer:
    """
    Visualizes models from a file or string buffer in Drake Visualizer,
    meldis, or MeshCat.

    To use this class to visualize model(s), create an instance with
    any desired options, add any models, and then call Run()::

      visualizer = ModelVisualizer(browser_new=True)
      visualizer.AddModels(filename)
      visualizer.Run()

    The class also provides a `parser()` method to allow more complex
    Parser handling, e.g. adding a model from a string::

      visualizer.parser().AddModelsFromString(buffer_containing_model, 'sdf')

    This class may also be run as a standalone command-line tool using the
    ``pydrake.visualization.model_visualizer`` script, or via
    ``bazel run //tools:model_visualizer``.
    """
    # Note: this class uses C++ method names to ease future porting.

    def __init__(self, *,
                 visualize_frames=False,
                 triad_length=0.3,
                 triad_radius=0.005,
                 triad_opacity=0.9,
                 publish_contacts=True,
                 show_rgbd_sensor=False,
                 browser_new=False,
                 pyplot=False,
                 meshcat=None):
        """
        Initializes a ModelVisualizer.

        Args:
          visualize_frames: a flag that visualizes frames as triads for all
            links.
          triad_length: the length of visualization triads.
          triad_radius: the radius of visualization triads.
          triad_opacity: the opacity of visualization triads.
          publish_contacts: a flag for VisualizationConfig.
          show_rgbd_sensor: when True, adds an RgbdSensor to the scene and pops
             up a local preview window of the rgb image. At the moment, the
             image display uses a native window so will not work in a remote or
             cloud runtime environment.

          browser_new: a flag that will open the MeshCat display in a new
            browser window during Run().
          pyplot: a flag that will open a pyplot figure for rendering using
            PlanarSceneGraphVisualizer.

          meshcat: an existing Meshcat instance to re-use instead of creating
            a new instance. Useful in, e.g., Python notebooks.
        """
        self._visualize_frames = visualize_frames
        self._triad_length = triad_length
        self._triad_radius = triad_radius
        self._triad_opacity = triad_opacity
        self._publish_contacts = publish_contacts
        self._show_rgbd_sensor = show_rgbd_sensor
        self._browser_new = browser_new
        self._pyplot = pyplot
        self._meshcat = meshcat

        # This is the list of loaded models, to enable the Reload button.
        # If set to None, it means that we won't support reloading because
        # the user might have added models outside of our purview. Each item
        # in the list contains whatever kwargs we passed to AddModels().
        self._added_models = list()

        # This is set to a non-None value iff our Meshcat has a reload button.
        self._reload_button_name = None

        # The builder is set to None during Finalize(), though during a Reload
        # it will be temporarily resurrected.
        self._builder = RobotDiagramBuilder()
        self._builder.parser().SetAutoRenaming(True)

        # The following fields are set non-None during Finalize().
        self._original_package_map = None
        self._diagram = None  # This will be a planning.RobotDiagram.
        self._sliders = None
        self._context = None

    def _check_rep(self, *, finalized):
        """
        Checks that our self members are consistent with the provided expected
        state of finalization.
        """
        if not finalized:
            # The builder is alive.
            assert self._builder is not None
            # The meshcat might or might not exist yet.
            # Everything else is dead.
            assert self._original_package_map is None
            assert self._diagram is None
            assert self._sliders is None
            assert self._context is None
        else:
            # The builder is dead.
            assert self._builder is None
            # Everything else is alive.
            assert self._meshcat is not None
            assert self._original_package_map is not None
            assert self._diagram is not None
            assert self._sliders is not None
            assert self._context is not None

    @staticmethod
    def _get_constructor_defaults():
        """
        Returns a dict of the default values used in our constructor's named
        keyword arguments (for any non-None values); this helps our companion
        main() function share those same defaults.
        """
        result = dict()
        prototype = ModelVisualizer()
        for name in [
                "visualize_frames",
                "triad_length",
                "triad_radius",
                "triad_opacity",
                "publish_contacts",
                "show_rgbd_sensor",
                "browser_new",
                "pyplot"]:
            value = getattr(prototype, f"_{name}")
            assert value is not None
            result[name] = value
        return result

    def package_map(self):
        """
        Returns the PackageMap being used during parsing. Users should add
        entries here to be able to load models from non-Drake packages.

        This method cannot be used after Finalize is called.
        """
        if self._builder is None:
            raise ValueError("Finalize has already been called.")
        self._check_rep(finalized=False)
        # It's safe to let the user change the package map. We'll make a copy
        # of it during Finalize().
        return self._builder.parser().package_map()

    def parser(self):
        """
        (Advanced) Returns a Parser that will load models into this visualizer.

        Prefer to use package_map() and AddModels() to load models, instead of
        this method.

        Calling this method will disable the "Reload" button in the visualizer,
        because we can no longer determine the scope of what to reload.

        This method cannot be used after Finalize is called.
        """
        if self._builder is None:
            raise ValueError("Finalize has already been called.")
        self._check_rep(finalized=False)
        # We can't easily know what the user is going to do with the parser,
        # so we need to disable model reloading once they access it.
        self._added_models = None
        return self._builder.parser()

    def meshcat(self):
        """
        Returns the Meshcat object this visualizer is plugged into.
        If none was provided in the constructor, this creates one on demand.
        """
        if self._meshcat is None:
            self._meshcat = StartMeshcat()
        return self._meshcat

    def AddModels(self, filename: Path = None, *, url: str = None):
        """
        Adds all models found in an input file (or url).

        This can be called multiple times, until the object is finalized.

        Args:
          filename: the name of a file containing one or more models.
          url: the package:// URL containing one or more models.

        Exactly one of filename or url must be non-None.
        """
        if self._builder is None:
            raise ValueError("Finalize has already been called.")
        if sum([filename is not None, url is not None]) != 1:
            raise ValueError("Must provide either filename= or url=")
        self._check_rep(finalized=False)
        if filename is not None:
            kwargs = dict(file_name=filename)
        else:
            assert url is not None
            kwargs = dict(url=url)
        self._builder.parser().AddModels(**kwargs)
        if self._added_models is not None:
            self._added_models.append(kwargs)

    def Finalize(self, position=None):
        """
        Finalizes the object and sets up the visualization using the provided
        options once models have been added.

        Args:
          position: a list of slider positions for the model(s); must match
            the number of positions in all model(s), including the 7 positions
            corresponding to any model bases that aren't welded to the world.
        """
        self._check_rep(finalized=False)
        if self._builder is None:
            raise RuntimeError("Finalize has already been called.")

        if self._visualize_frames:
            # Find all the frames and draw them.
            # The frames are drawn using the parsed length.
            # The world frame is drawn thicker than the rest.
            inspector = self._builder.scene_graph().model_inspector()
            for frame_id in inspector.GetAllFrameIds():
                world_id = self._builder.scene_graph().world_frame_id()
                radius = self._triad_radius * (
                    3 if frame_id == world_id else 1
                    )
                AddFrameTriadIllustration(
                    plant=self._builder.plant(),
                    scene_graph=self._builder.scene_graph(),
                    frame_id=frame_id,
                    length=self._triad_length,
                    radius=radius,
                    opacity=self._triad_opacity,
                )

        # Add a model that will provide rgbd pose sliders automatically when we
        # add JointSliders later on.
        if self._show_rgbd_sensor:
            camera_sliders, = self._builder.parser().AddModels(url=(
                "package://drake/bindings/pydrake/visualization/"
                "_rgbd_camera_sliders.dmd.yaml"))
            # Remove the perception role from the new geometry; we don't want
            # the RgbdSensor to render it. TODO(#13689) The file should itself
            # opt-out of the perception role, so we don't need to mop up here.
            inspect = self._builder.scene_graph().model_inspector()
            for frame_id in inspect.GetAllFrameIds():
                if inspect.GetFrameGroup(frame_id) == camera_sliders:
                    self._builder.scene_graph().RemoveRole(
                        role=Role.kPerception, frame_id=frame_id,
                        source_id=self._builder.plant().get_source_id())

        self._builder.plant().Finalize()

        # (Re-)initialize the meshcat instance, creating one if needed.
        self.meshcat()
        self._meshcat.Delete()
        self._meshcat.DeleteAddedControls()

        # We want to place the Reload Model Files button far away from the
        # Stop Running button, hence the work to do this here.
        if self._added_models:
            self._reload_button_name = "Reload Model Files"
            self._meshcat.AddButton(self._reload_button_name)

        # Connect to drake_visualizer, meldis, and meshcat.
        # Meldis and meshcat provide simultaneous visualization of
        # illustration and proximity geometry.
        ApplyVisualizationConfig(
            config=VisualizationConfig(
                publish_contacts=self._publish_contacts,
                enable_alpha_sliders=True),
            plant=self._builder.plant(),
            scene_graph=self._builder.scene_graph(),
            builder=self._builder.builder(),
            meshcat=self._meshcat)

        # Add a render camera so we can show role=perception images.
        if self._show_rgbd_sensor:
            camera_config = CameraConfig(width=1440, height=1080)
            camera_config.name = "preview"
            camera_config.X_PB.base_frame = "_rgbd_camera_sliders::pinhole"
            camera_config.z_far = 3  # Show 3m of frustum.
            camera_config.fps = 1.0  # Ignored -- we're not simulating.
            is_unit_test = "TEST_SRCDIR" in os.environ
            camera_config.show_rgb = not is_unit_test  # Pop up a local window.
            ApplyCameraConfig(
                config=camera_config,
                builder=self._builder.builder())
            camera_sensor = self._builder.builder().GetSubsystemByName(
                "rgbd_sensor_preview")
            camera_publisher = self._builder.builder().GetSubsystemByName(
                "LcmPublisherSystem(DRAKE_RGBD_CAMERA_IMAGES_preview)")
            # Export the preview camera image output port for later use.
            self._builder.builder().ExportOutput(
                camera_sensor.GetOutputPort("color_image"), "preview_image")
            # Disable LCM image transmission. It has a non-trivial cost, and
            # at the moment Meldis can't display LCM images anyway.
            self._builder.builder().RemoveSystem(camera_publisher)

        # Add joint sliders to meshcat.
        # TODO(trowell-tri) Restoring slider values depends on the slider
        # names remaining consistent across the reload; currently the
        # JointSlider code names sliders by joint name and position suffix
        # and adds a model name suffix when those names aren't unique, e.g.
        # when the same model appears multiple times. Thus if the set of
        # models changes to cause or eliminate such a name collision then
        # slider values won't be fully restored. It would probably be better to
        # be able to configure the JointSliders to always use fully-qualified
        # names on demand, especially once the size of the control panel is
        # adjustable so that the slider names are better visible.
        self._sliders = self._builder.builder().AddNamedSystem(
            "joint_sliders", JointSliders(meshcat=self._meshcat,
                                          plant=self._builder.plant()))

        # Connect to PyPlot.
        if self._pyplot:
            ConnectPlanarSceneGraphVisualizer(
                self._builder.builder(), self._builder.scene_graph())

        self._original_package_map = copy.copy(
            self._builder.parser().package_map())
        self._diagram = self._builder.Build()
        self._builder = None
        self._context = self._diagram.CreateDefaultContext()

        # We don't just test 'position' because NumPy does weird things with
        # the truth values of arrays.
        if position is not None and len(position) > 0:
            self._raise_if_invalid_positions(position)
            self._diagram.plant().SetPositions(
                self._diagram.plant().GetMyContextFromRoot(self._context),
                position)
            self._sliders.SetPositions(position)

        # Use Simulator to dispatch initialization events.
        # TODO(eric.cousineau): Simplify as part of #13776 (was #10015).
        Simulator(self._diagram).Initialize()
        # Publish draw messages with current state.
        self._diagram.ForcedPublish(self._context)

        # Visualize the camera frustum.
        if self._show_rgbd_sensor:
            camera_path = "/drake/illustration/_rgbd_camera_sliders/pinhole"
            frustum_path = f"{camera_path}/frustum"
            (vertices, faces) = self._camera_config_to_frustum(camera_config)
            self._meshcat.SetTriangleMesh(
                path=frustum_path, vertices=vertices, faces=faces,
                rgba=Rgba(1.0, 0.33, 0, 0.2))
            self._meshcat.SetTriangleMesh(
                path=f"{frustum_path}/wire", vertices=vertices, faces=faces,
                rgba=Rgba(0.5, 0.5, 0.5), wireframe=True)

        self._check_rep(finalized=True)

    @staticmethod
    def _camera_config_to_frustum(camera: CameraConfig):
        """
        Returns a mesh as (vertices, faces) to visualize the given camera's
        frustum.
        """
        distance = camera.z_far
        width = 0.5 * camera.width * distance / camera.focal_x()
        height = 0.5 * camera.height * distance / camera.focal_y()
        vertices = np.array([
            [0.0, 0.0, 0.0],
            [+width, +height, distance],
            [+width, -height, distance],
            [-width, -height, distance],
            [-width, +height, distance],
        ]).T
        faces = np.array([
            [0, 1, 2],
            [0, 2, 3],
            [0, 3, 4],
            [0, 4, 1],
        ]).T
        return (vertices, faces)

    def _reload(self):
        """
        Re-creates the Diagram using the same sequence of calls to AddModels
        as the user performed. In effect, this will refresh the visualizer to
        show any changes the user made on disk to their models.
        """
        self._check_rep(finalized=True)
        assert self._added_models is not None

        # Clear out the old diagram.
        self._diagram = None
        self._sliders = None
        self._context = None
        self._remove_traffic_cone()

        # Populate the diagram builder again with the same packages and models.
        self._builder = RobotDiagramBuilder()
        self._builder.parser().SetAutoRenaming(True)
        self._builder.parser().package_map().AddMap(self._original_package_map)
        try:
            for kwargs in self._added_models:
                self._builder.parser().AddModels(**kwargs)
            logging.getLogger("drake").info(f"Reload was successful")
        except BaseException as e:
            # If there's a parsing error, show it; don't crash.
            logging.getLogger("drake").error(e)
            logging.getLogger("drake").warning(
                f"Click '{self._reload_button_name}' to try again")
            # Clear the display to help indicate the failure to the user.
            self._builder = RobotDiagramBuilder()
            self._builder.parser().package_map().AddMap(
                self._original_package_map)
            self._add_traffic_cone()
        self._original_package_map = None

        # Finalize the rest of the systems and widgets.
        self.Finalize()

    def Run(self, position=None, loop_once=False):
        """
        Runs the model. If Finalize() hasn't already been explicitly called
        then the object will be finalized first.

        Will iterate once and exit if `loop_once` is True, otherwise will
        loop until the user quits.

        Args:
          position: an ndarray-like list of slider positions for the model(s);
            must match the number of positions in all model(s), including the
            7 positions corresponding to any model bases that aren't welded to
            the world.
          loop_once: a flag that exits the evaluation loop after one pass.
        """
        if self._builder is not None:
            self.Finalize(position=position)
        else:
            self._check_rep(finalized=True)
            if position is not None and len(position) > 0:
                self._raise_if_invalid_positions(position)
                self._diagram.plant().SetPositions(
                    self._diagram.plant().GetMyContextFromRoot(self._context),
                    position)
                self._sliders.SetPositions(position)
                self._diagram.ForcedPublish(self._context)

        # Everything is finally fully configured. We can open the window now.
        # TODO(jwnimmer-tri) The browser_new config knob would probably make
        # more sense as an argument to Run() vs an argument to our constructor.
        if self._browser_new:
            self._browser_new = False
            url = self._meshcat.web_url()
            _webbrowser_open(url=url, new=True)

        # Wait for the user to cancel us.
        stop_button_name = "Stop Running"
        if not loop_once:
            logging.getLogger("drake").info(
                f"Click '{stop_button_name}' or press Esc to quit")

        last_camera_time = 0
        try:
            self._meshcat.AddButton(stop_button_name, "Escape")

            def has_clicks(button_name):
                if not button_name:
                    return False
                return self._meshcat.GetButtonClicks(button_name) > 0

            while True:
                # Refresh the relatively expensive preview camera at a slower
                # rate (2 Hz) than everything else (32 Hz).
                if self._show_rgbd_sensor:
                    if time.time() > (last_camera_time + 0.5):
                        last_camera_time = time.time()
                        self._diagram.GetOutputPort("preview_image").Eval(
                            self._context)
                time.sleep(1 / 32.0)
                if has_clicks(self._reload_button_name):
                    self._meshcat.DeleteButton(stop_button_name)
                    slider_values = self._get_slider_values()
                    self._reload()
                    self._set_slider_values(slider_values)
                    self._meshcat.AddButton(stop_button_name, "Escape")
                q = self._sliders.get_output_port().Eval(
                    self._sliders.GetMyContextFromRoot(self._context))
                self._diagram.plant().SetPositions(
                    self._diagram.plant().GetMyContextFromRoot(self._context),
                    q)
                self._diagram.ForcedPublish(self._context)
                if loop_once or has_clicks(stop_button_name):
                    break
        except KeyboardInterrupt:
            pass

        self._meshcat.DeleteButton(stop_button_name)
        if self._reload_button_name is not None:
            self._meshcat.DeleteButton(self._reload_button_name)
            self._reload_button_name = None

    def _raise_if_invalid_positions(self, position):
        """
        Validate the position argument.

        Raises:
          ValueError: if the length of the position list does not match
        the number of positions in the plant.
        """
        assert self._diagram is not None
        actual = len(position)
        expected = self._diagram.plant().num_positions()
        if actual != expected:
            raise ValueError(
                f"Number of passed positions ({actual}) does not match the "
                f"number in the model ({expected}).")

    def _get_slider_values(self):
        """Returns a map of slider names to current values."""
        return {name: self._meshcat.GetSliderValue(name)
                for name in self._meshcat.GetSliderNames()}

    def _set_slider_values(self, slider_values):
        """
        Sets current sliders to the values found in the slider_values dict.

        Current sliders not in the passed map -- or values in the map but
        which not longer exist in the GUI -- are ignored.
        """
        current_names = self._meshcat.GetSliderNames()
        for old_name, old_value in slider_values.items():
            if old_name in current_names:
                self._meshcat.SetSliderValue(old_name, old_value)

    def _add_traffic_cone(self):
        """Adds a traffic cone to the scene, indicating a parsing error."""
        base_width = 0.4
        base_thickness = 0.01
        base = Box(base_width, base_width, base_thickness)
        cone_height = 0.6
        cone_radius = 0.75 * (base_width / 2)
        cone = MeshcatCone(cone_height, cone_radius, cone_radius)

        path = "/PARSE_ERROR"
        orange = Rgba(1.0, 0.33, 0)
        self._meshcat.SetObject(path=f"{path}/base", shape=base, rgba=orange)
        self._meshcat.SetObject(path=f"{path}/cone", shape=cone, rgba=orange)
        self._meshcat.SetTransform(f"{path}/base", RigidTransform(
            [0, 0, base_thickness * 0.5]))
        self._meshcat.SetTransform(f"{path}/cone", RigidTransform(
            RotationMatrix.MakeYRotation(np.pi),
            [0, 0, base_thickness + cone_height]))

    def _remove_traffic_cone(self):
        """Removes the traffic cone from the scene."""
        self._meshcat.Delete("/PARSE_ERROR")
