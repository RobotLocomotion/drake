import copy
from enum import Enum
import time
from webbrowser import open as _webbrowser_open

import numpy as np

from pydrake.common.deprecation import deprecated
from pydrake.geometry import (
    Cylinder,
    GeometryInstance,
    MakePhongIllustrationProperties,
    StartMeshcat,
)
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.meshcat import JointSliders
from pydrake.planning import RobotDiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.planar_scenegraph_visualizer import (
    ConnectPlanarSceneGraphVisualizer,
)
from pydrake.visualization import (
    VisualizationConfig,
    ApplyVisualizationConfig,
)


class RunResult(Enum):
    """This class is deprecated and will be removed on or after 2023-07-01."""
    KEEP_GOING = 0  # Note that this is never returned but is used internally.
    LOOP_ONCE = 1
    STOPPED = 2
    RELOAD = 3


class ModelVisualizer:
    """
    Visualizes models from a file or string buffer in Drake Visualizer,
    meldis, or MeshCat.

    To use this class to visualize model(s), create an instance with
    any desired options, add any models, and then call Run()::

      visualizer = ModelVisualizer(browser_new=True)
      visualizer.AddModels(filename)
      visualizer.Run()

    The class also provides a `parser` property to allow more complex
    Parser handling, e.g. adding a model from a string::

      visualizer.parser.AddModelsFromString(buffer_containing_model, 'sdf')

    This class may also be run as a standalone command-line tool using the
    ``pydrake.visualization.model_visualizer`` script, or via
    ``bazel run //tools:model_visualizer``.
    """
    # Note: this class uses C++ method names to ease future porting.

    def __init__(self, *,
                 visualize_frames=False,
                 triad_length=0.5,
                 triad_radius=0.01,
                 triad_opacity=1,
                 publish_contacts=True,
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

          browser_new: a flag that will open the MeshCat display in a new
            browser window.
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
        self._browser_new = browser_new
        self._pyplot = pyplot
        self._meshcat = meshcat

        # This is the list of loaded models, to enable the Reload button.
        # If set to None, it means that we won't support reloading because
        # the user might have added models outside of our purview.
        self._model_filenames = list()

        # This is set to a non-None value iff our Meshcat has a reload button.
        self._reload_button_name = None

        # The builder is set to None during Finalize(), though during a Reload
        # it will be temporarily resurrected.
        self._builder = RobotDiagramBuilder()

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
        self._model_filenames = None
        return self._builder.parser()

    def meshcat(self):
        """
        Returns the Meshcat object this visualizer is plugged into.
        If none was provided in the constructor, this creates one on demand.
        """
        if self._meshcat is None:
            self._meshcat = StartMeshcat()
        return self._meshcat

    def AddModels(self, filename):
        """
        Adds all models found in an input file.

        This can be called multiple times, until the object is finalized.

        Args:
          filename: the name of a file containing one or more models.
        """
        if self._builder is None:
            raise ValueError("Finalize has already been called.")
        self._check_rep(finalized=False)
        self._builder.parser().AddModels(filename)
        if self._model_filenames is not None:
            self._model_filenames.append(filename)

    def Finalize(self, position=None):
        """
        Finalizes the object and sets up the visualization using the provided
        options once models have been added.

        Args:
          position: a list of slider positions for the model(s); must match
            the number of positions in all model(s), including the 7 positions
            corresponding to any model bases that aren't welded to the world.
        """
        if self._builder is None:
            raise RuntimeError("Finalize has already been called.")
        self._check_rep(finalized=False)

        if self._visualize_frames:
            # Find all the frames and draw them using _add_triad().
            # The frames are drawn using the parsed length.
            # The world frame is drawn thicker than the rest.
            inspector = self._builder.scene_graph().model_inspector()
            for frame_id in inspector.GetAllFrameIds():
                world_id = self._builder.scene_graph().world_frame_id()
                radius = self._triad_radius * (
                    3 if frame_id == world_id else 1
                    )
                self._add_triad(
                    frame_id,
                    length=self._triad_length,
                    radius=radius,
                    opacity=self._triad_opacity,
                )

        self._builder.plant().Finalize()

        # (Re-)initialize the meshcat instance, creating one if needed.
        self.meshcat()
        self._meshcat.Delete()
        self._meshcat.DeleteAddedControls()

        # We want to place this button far away from the Stop Running button,
        # hence the work to do this here.
        if self._model_filenames:
            self._reload_button_name = "Reload model files"
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

        # Add joint sliders to meshcat.
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
            self._sliders.SetPositions(position)
            self._diagram.plant().SetPositions(
                self._diagram.plant().GetMyContextFromRoot(self._context),
                position)

        if self._browser_new:
            self._browser_new = False
            url = self._meshcat.web_url()
            _webbrowser_open(url=url, new=True)

        # Use Simulator to dispatch initialization events.
        # TODO(eric.cousineau): Simplify as part of #13776 (was #10015).
        Simulator(self._diagram).Initialize()
        # Publish draw messages with current state.
        self._diagram.ForcedPublish(self._context)

        # Disable the proximity geometry at the start; it can be enabled by
        # the checkbox in the meshcat controls.
        self._meshcat.SetProperty("proximity", "visible", False)

        self._check_rep(finalized=True)

    def _reload(self):
        """
        Re-creates the Diagram using the same sequence of calls to AddModels
        as the user performed. In effect, this will refresh the visualizer to
        show any changes the user made on disk to their models.
        """
        self._check_rep(finalized=True)
        assert self._model_filenames is not None

        # Clear out the old diagram.
        self._diagram = None
        self._sliders = None
        self._context = None

        # Populate the diagram builder again with the same packages and models.
        self._builder = RobotDiagramBuilder()
        self._builder.parser().package_map().AddMap(self._original_package_map)
        self._original_package_map = None
        for filename in self._model_filenames:
            self._builder.parser().AddModels(filename)

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
                self._diagram.plant().SetPositions(
                    self._diagram.plant().GetMyContextFromRoot(self._context),
                    position)
                self._sliders.SetPositions(position)
                self._diagram.ForcedPublish(self._context)

        # Wait for the user to cancel us.
        stop_button_name = "Stop Running"
        if not loop_once:
            print(f"Click '{stop_button_name}' or press Esc to quit")

        loop_result = RunResult.KEEP_GOING
        try:
            self._meshcat.AddButton(stop_button_name, "Escape")

            def has_clicks(button_name):
                if not button_name:
                    return False
                return self._meshcat.GetButtonClicks(button_name) > 0

            while loop_result == RunResult.KEEP_GOING:
                time.sleep(1 / 32.0)
                q = self._sliders.get_output_port().Eval(
                    self._sliders.GetMyContextFromRoot(self._context))
                self._diagram.plant().SetPositions(
                    self._diagram.plant().GetMyContextFromRoot(self._context),
                    q)
                self._diagram.ForcedPublish(self._context)
                if loop_once:
                    loop_result = RunResult.LOOP_ONCE
                if has_clicks(stop_button_name):
                    loop_result = RunResult.STOPPED
                if has_clicks(self._reload_button_name):
                    self._meshcat.DeleteButton(stop_button_name)
                    self._reload()
                    self._meshcat.AddButton(stop_button_name, "Escape")
        except KeyboardInterrupt:
            loop_result = RunResult.STOPPED

        self._meshcat.DeleteButton(stop_button_name)
        if self._reload_button_name:
            self._meshcat.DeleteButton(self._reload_button_name)
            self._reload_button_name = None

        return loop_result

    @deprecated("Use Run() instead.", date="2023-07-01")
    def RunWithReload(self, *args, **kwargs):
        """
        (Deprecated.) The reload feature is enabled by default during Run().
        """
        return self.Run(*args, **kwargs)

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

    def _add_triad(
        self,
        frame_id,
        *,
        length,
        radius,
        opacity,
        X_FT=RigidTransform(),
        name="frame",
    ):
        """
        Adds illustration geometry representing the coordinate frame, with
        the x-axis drawn in red, the y-axis in green and the z-axis in blue.
        The axes point in +x, +y and +z directions, respectively.
        Based on [code permalink](https://github.com/RussTedrake/manipulation/blob/5e59811/manipulation/scenarios.py#L367-L414).# noqa
        Args:
          frame_id: a geometry::frame_id registered with scene_graph.
          length: the length of each axis in meters.
          radius: the radius of each axis in meters.
          opacity: the opacity of the coordinate axes, between 0 and 1.
          X_FT: a RigidTransform from the triad frame T to the frame_id frame F
          name: the added geometry will have names name + " x-axis", etc.
        """
        # x-axis
        X_TG = RigidTransform(
            RotationMatrix.MakeYRotation(np.pi / 2),
            [length / 2.0, 0, 0],
        )
        geom = GeometryInstance(
            X_FT.multiply(X_TG), Cylinder(radius, length), name + " x-axis"
        )
        geom.set_illustration_properties(
            MakePhongIllustrationProperties([1, 0, 0, opacity])
        )
        self._builder.scene_graph().RegisterGeometry(
            self._builder.plant().get_source_id(), frame_id, geom)

        # y-axis
        X_TG = RigidTransform(
            RotationMatrix.MakeXRotation(np.pi / 2),
            [0, length / 2.0, 0],
        )
        geom = GeometryInstance(
            X_FT.multiply(X_TG), Cylinder(radius, length), name + " y-axis"
        )
        geom.set_illustration_properties(
            MakePhongIllustrationProperties([0, 1, 0, opacity])
        )
        self._builder.scene_graph().RegisterGeometry(
            self._builder.plant().get_source_id(), frame_id, geom)

        # z-axis
        X_TG = RigidTransform([0, 0, length / 2.0])
        geom = GeometryInstance(
            X_FT.multiply(X_TG), Cylinder(radius, length), name + " z-axis"
        )
        geom.set_illustration_properties(
            MakePhongIllustrationProperties([0, 0, 1, opacity])
        )
        self._builder.scene_graph().RegisterGeometry(
            self._builder.plant().get_source_id(), frame_id, geom)
