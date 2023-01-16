from enum import Enum
import time
import webbrowser

import numpy as np

from pydrake.geometry import (
    Cylinder,
    GeometryInstance,
    MakePhongIllustrationProperties,
    StartMeshcat,
)
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.meshcat import JointSliders
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.planar_scenegraph_visualizer import (
    ConnectPlanarSceneGraphVisualizer,
)
from pydrake.visualization import (
    VisualizationConfig,
    ApplyVisualizationConfig,
)


class RunResult(Enum):
    """Result code indicating how Run() terminated."""
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

        # This is both a button label and a flag to enable the button.
        self._reload_button_name = None
        self._model_filenames = list()

        # The following fields remain valid for this object's lifetime after
        # Finalize() has been called.
        self._diagram = None
        self._sliders = None
        self._context = None
        self._plant_context = None

        # The builder, scene_graph, and parser become invalid after Finalize().
        # The plant remains valid for this object's lifetime.
        self._builder = DiagramBuilder()
        self._plant, self._scene_graph = AddMultibodyPlantSceneGraph(
            self._builder, time_step=0.0)
        self._parser = Parser(self._plant)

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

    def parser(self):
        """
        Returns a Parser that will load models into this visualizer.

        This method cannot be used after Finalize is called.
        """
        assert self._parser is not None, "Finalize has already been called."
        return self._parser

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
        self._parser.AddModels(filename)
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
        assert all([x is not None for x in (
            self._plant,
            self._scene_graph,
            self._parser)])
        assert all([x is None for x in (
            self._diagram,
            self._sliders,
            self._context,
            self._plant_context)])

        if self._visualize_frames:
            # Find all the frames and draw them using _add_triad().
            # The frames are drawn using the parsed length.
            # The world frame is drawn thicker than the rest.
            inspector = self._scene_graph.model_inspector()
            for frame_id in inspector.GetAllFrameIds():
                radius = self._triad_radius * (
                    3 if frame_id == self._scene_graph.world_frame_id() else 1
                    )
                self._add_triad(
                    frame_id,
                    length=self._triad_length,
                    radius=radius,
                    opacity=self._triad_opacity,
                )

        self._plant.Finalize()

        # (Re-)initialize the meshcat instance, creating one if needed.
        self.meshcat()
        self._meshcat.Delete()
        self._meshcat.DeleteAddedControls()

        # We want to place this button far away from the Stop Running button,
        # hence the work to do this here.
        if self._reload_button_name:
            self._meshcat.AddButton(self._reload_button_name)

        # Connect to drake_visualizer, meldis, and meshcat.
        # Meldis and meshcat provide simultaneous visualization of
        # illustration and proximity geometry.
        ApplyVisualizationConfig(
            config=VisualizationConfig(
                publish_contacts=self._publish_contacts,
                enable_alpha_sliders=True),
            plant=self._plant,
            scene_graph=self._scene_graph,
            builder=self._builder,
            meshcat=self._meshcat)

        # Add joint sliders to meshcat.
        self._sliders = self._builder.AddNamedSystem(
            "joint_sliders", JointSliders(meshcat=self._meshcat,
                                          plant=self._plant))

        # Connect to PyPlot.
        if self._pyplot:
            ConnectPlanarSceneGraphVisualizer(self._builder, self._scene_graph)

        self._diagram = self._builder.Build()
        self._context = self._diagram.CreateDefaultContext()
        self._plant_context = self._plant.GetMyContextFromRoot(self._context)

        # We don't just test 'position' because NumPy does weird things with
        # the truth values of arrays.
        if position is not None and len(position) > 0:
            self._raise_if_invalid_positions(position)
            self._plant.SetPositions(self._plant_context, position)
            self._sliders.SetPositions(position)

        if self._browser_new:
            url = self._meshcat.web_url()
            webbrowser.open(url=url, new=self._browser_new)

        # Use Simulator to dispatch initialization events.
        # TODO(eric.cousineau): Simplify as part of #13776 (was #10015).
        Simulator(self._diagram).Initialize()
        # Publish draw messages with current state.
        self._diagram.ForcedPublish(self._context)

        # Disable the proximity geometry at the start; it can be enabled by
        # the checkbox in the meshcat controls.
        self._meshcat.SetProperty("proximity", "visible", False)

        self._builder = None
        self._scene_graph = None
        self._parser = None

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

        Returns:
          A RunResult code indicating why the method stopped.
        """
        if self._builder:
            self.Finalize(position=position)
        elif position is not None and len(position) > 0:
            self._plant.SetPositions(self._plant_context, position)
            self._sliders.SetPositions(position)
            self._diagram.ForcedPublish(self._context)

        assert all([x is None for x in (
            self._builder,
            self._scene_graph,
            self._parser)])
        assert all([x is not None for x in (
            self._plant,
            self._diagram,
            self._sliders,
            self._context,
            self._plant_context,
            self._meshcat)])

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

            sliders_context = self._sliders.GetMyContextFromRoot(self._context)
            while loop_result == RunResult.KEEP_GOING:
                time.sleep(1 / 32.0)
                q = self._sliders.get_output_port().Eval(sliders_context)
                self._plant.SetPositions(self._plant_context, q)
                self._diagram.ForcedPublish(self._context)
                if loop_once:
                    loop_result = RunResult.LOOP_ONCE
                if has_clicks(stop_button_name):
                    loop_result = RunResult.STOPPED
                if has_clicks(self._reload_button_name):
                    loop_result = RunResult.RELOAD
        except KeyboardInterrupt:
            loop_result = RunResult.STOPPED
        finally:
            self._meshcat.DeleteButton(stop_button_name)
            if self._reload_button_name:
                self._meshcat.DeleteButton(self._reload_button_name)

        return loop_result

    def RunWithReload(self, position=None):
        """
        Runs the model with an option to reload.
        Neither Finalize() nor Run() may be called before this method.

        Will iterate until the user clicks the Stop Running button. All model
        files will be reloaded and meshcat will be reset if the reload button
        is clicked. If pyplot is being used, clicking reload will create a
        new pyplot window.

        Only models added using AddModels() will be properly reloaded.
        If the parser() was accessed to add models directly to the parser
        then those models will not be properly reloaded by this method.

        Args:
          position: an ndarray-like list of slider positions for the model(s);
            must match the number of positions in all model(s), including the
            7 positions corresponding to any model bases that aren't welded to
            the world.
        """
        # This method uses this outer visualizer (self) to track options
        # and manage meshcat (and its window, if applicable). We then
        # create an inner visualizer each time through the reload loop.

        # This outer visualizer must not have been Finalized or Run.
        if self._builder is None:
            raise RuntimeError("Finalize has already been called.")
        assert all([x is not None for x in (
            self._plant,
            self._scene_graph,
            self._parser)])
        assert all([x is None for x in (
            self._diagram,
            self._sliders,
            self._context,
            self._plant_context)])

        # We don't just test 'position' because NumPy does weird things with
        # the truth values of arrays.
        if position is not None and len(position) > 0:
            # If we have a position list to validate, finalize the outer
            # visualizer's plant only so we can validate positions.
            self._plant.Finalize()
            self._raise_if_invalid_positions(position)

        if self._browser_new:
            url = self.meshcat().web_url()
            webbrowser.open(url=url, new=self._browser_new)

        model_suffix = "s" if len(self._model_filenames) != 1 else ""
        reload_button_name = "Reload Model File%s" % model_suffix

        while True:
            inner_visualizer = ModelVisualizer(
                visualize_frames=self._visualize_frames,
                triad_length=self._triad_length,
                triad_radius=self._triad_radius,
                triad_opacity=self._triad_opacity,
                browser_new=False,
                pyplot=self._pyplot,
                meshcat=self._meshcat)

            inner_visualizer._reload_button_name = reload_button_name
            for filename in self._model_filenames:
                inner_visualizer.AddModels(filename)

            # Finalize the inner visualizer outside of Run so that we can
            # make sure the positions we saved from the prior iteration
            # are still usable with the reloaded model. This is necessary
            # so that changes in the number of joints doesn't just exit.
            inner_visualizer.Finalize()

            # We don't just test 'position' because NumPy does weird things
            # with the truth values of arrays.
            if position is not None and len(position) > 0:
                # Don't exit if the number of positions in the model changes.
                # TODO(trowell-tri) Examine the plant more thoroughly
                # and properly restore position values to the same joints.
                if inner_visualizer._plant.num_positions() != len(position):
                    position = None

            result = inner_visualizer.Run(position)
            if result != RunResult.RELOAD:
                return

            position = inner_visualizer._sliders.get_output_port().Eval(
                inner_visualizer._sliders.GetMyContextFromRoot(
                    inner_visualizer._context))

    def _raise_if_invalid_positions(self, position):
        """
        Validate the position argument.

        Raises:
          ValueError: if the length of the position list does not match
        the number of positions in the plant.
        """
        assert self._plant is not None
        if self._plant.num_positions() != len(position):
            message = ("Number of passed positions (%d) does not match "
                       "the number in the model (%d).")
            raise ValueError(
                message % (len(position), self._plant.num_positions()))

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
        self._scene_graph.RegisterGeometry(
            self._plant.get_source_id(), frame_id, geom)

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
        self._scene_graph.RegisterGeometry(
            self._plant.get_source_id(), frame_id, geom)

        # z-axis
        X_TG = RigidTransform([0, 0, length / 2.0])
        geom = GeometryInstance(
            X_FT.multiply(X_TG), Cylinder(radius, length), name + " z-axis"
        )
        geom.set_illustration_properties(
            MakePhongIllustrationProperties([0, 0, 1, opacity])
        )
        self._scene_graph.RegisterGeometry(
            self._plant.get_source_id(), frame_id, geom)
