r"""Loads a model file (*.sdf or *.urdf) and then displays all models found
in all available visualizers (MeshCat, meldis/drake visualizer).

Joint sliders are available in visualizers that support them by clicking on
"Open Controls" in top right corner.

To have the MeshCat server automatically open in your browser, supply the
`--open-window` flag.

To build all necessary targets and see available command-line options:

    cd drake
    bazel build \
        //tools:drake_visualizer //manipulation/util:show_model

    ./bazel-bin/manipulation/util/show_model --help

Example usage:

    # Terminal 1
    ./bazel-bin/tools/drake_visualizer

    # Terminal 2 (wait for visualizers to start)
    ./bazel-bin/manipulation/util/show_model \
        --open-window \
        --position 1 0 0 0 0 0 0 0 1 0 0 0 0 0 \
        --find_resource \
        drake/manipulation/models/iiwa_description/iiwa7/iiwa7_with_box_collision.sdf

If your model has all of its data available in your source tree, then you can
remove the need for `--find_resource`:

    ./bazel-bin/manipulation/util/show_model \
        --open-window \
        ${PWD}/manipulation/models/iiwa_description/iiwa7/iiwa7_with_box_collision.sdf

If the model uses package path (e.g. "package://package_name/model_sdf.obj") to
refer to mesh files, you also have to provide the argument `--package_path`:
    ./bazel-bin/manipulation/util/show_model \
        --package_path multibody/parsing/test/box_package \
        multibody/parsing/test/box_package/sdfs/box.sdf

Note:
    The output of running ``show_model`` will include the URL of the MeshCat
server.

Note:
    If you use `bazel run` without `--find_resource`, it is highly encouraged
that you use absolute paths, as certain models may not be prerequisites of this
binary.
"""

import argparse
import os
import time
import webbrowser

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import (
    Cylinder,
    GeometryInstance,
    MakePhongIllustrationProperties,
    Meshcat,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Role,
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


class ModelVisualizer:
    """
    Visualizes models from a file or string buffer in Drake Visualizer,
    meldis, or MeshCat.

    To use this class to visualize model(s), create an instance with
    any desired options, add any models, and then call Run():

        visualizer = ModelVisualizer(browser_new=True)
        visualizer.AddModels(filename)
        visualizer.Run()

    The class also provides a `parser` property to allow more complex
    Parser handling, e.g. adding a model from a string:
        visualizer.parser.AddModelsFromString(buffer_containing_model, 'sdf')
    """
    # Note: this class uses C++ method names to ease future porting.

    def __init__(self, *,
                 visualize_frames=False,
                 triad_length=0.5,
                 triad_radius=0.01,
                 triad_opacity=1,
                 publish_contacts=True,
                 browser_new=False,
                 pyplot=False):
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
        """
        self._visualize_frames = visualize_frames
        self._triad_length = triad_length
        self._triad_radius = triad_radius
        self._triad_opacity = triad_opacity
        self._publish_contacts = publish_contacts
        self._browser_new = browser_new
        self._pyplot = pyplot

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

    @property
    def visualize_frames(self):
        return self._visualize_frames

    @property
    def triad_length(self):
        return self._triad_length

    @property
    def triad_radius(self):
        return self._triad_radius

    @property
    def triad_opacity(self):
        return self._triad_opacity

    @property
    def publish_contacts(self):
        return self._publish_contacts

    @property
    def browser_new(self):
        return self._browser_new

    @property
    def pyplot(self):
        return self._pyplot

    @property
    def parser(self):
        """
        The internal Parser instance.

        This property is only valid until Finalize is called.
        """
        assert self._parser is not None, "Finalize has already been called."
        return self._parser

    def AddModels(self, filename):
        """
        Adds all models found in an input file.

        This can be called multiple times, until the object is finalized.

        Args:
          filename: the name of a file containing one or more models.
        """
        self._parser.AddModels(filename)

    def Finalize(self, position=None):
        """
        Finalizes the object and sets up the visualization using the provided
        options once models have been added.

        Args:
          position: a list of slider positions for the model(s); must match
            the number of positions in all model(s), including the 7 positions
            corresponding to any model bases that aren't welded to the world.
        """
        assert self._builder is not None, "Finalize has already been called."

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

        # Connect to drake_visualizer or meldis. Meldis provides simultaneous
        # visualization of illustration and proximity geometry.
        ApplyVisualizationConfig(
            config=VisualizationConfig(
                publish_contacts=self._publish_contacts),
            plant=self._plant,
            scene_graph=self._scene_graph,
            builder=self._builder)

        # Connect to MeshCat for visualizing and interfacing w/ widgets.
        meshcat = Meshcat()
        # Add two visualizers: one to publish the "illustration" geometry and
        # another to publish the "collision" geometry.
        MeshcatVisualizer.AddToBuilder(
            self._builder, self._scene_graph, meshcat,
            MeshcatVisualizerParams(role=Role.kIllustration, prefix="visual"))
        MeshcatVisualizer.AddToBuilder(
            self._builder, self._scene_graph, meshcat,
            MeshcatVisualizerParams(role=Role.kProximity, prefix="collision"))
        self._sliders = self._builder.AddNamedSystem(
            "joint_sliders", JointSliders(meshcat=meshcat, plant=self._plant))

        if self._browser_new:
            url = meshcat.web_url()
            webbrowser.open(url=url, new=self._browser_new)

        # Connect to PyPlot.
        if self._pyplot:
            ConnectPlanarSceneGraphVisualizer(self._builder, self._scene_graph)

        self._diagram = self._builder.Build()
        self._context = self._diagram.CreateDefaultContext()
        self._plant_context = self._plant.GetMyContextFromRoot(self._context)

        if position:
            self._plant.SetPositions(self._plant_context, position)
            self._sliders.SetPositions(position)

        # Use Simulator to dispatch initialization events.
        # TODO(eric.cousineau): Simplify as part of #13776 (was #10015).
        Simulator(self._diagram).Initialize()
        # Publish draw messages with current state.
        self._diagram.Publish(self._context)

        # Disable the collision geometry at the start; it can be enabled by
        # the checkbox in the meshcat controls.
        meshcat.SetProperty("collision", "visible", False)

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
            the world. If Finalize() has already been called then this
            argument has no effect.
          loop_once: a flag that exits the evaluation loop after one pass.
        """
        if self._builder:
            self.Finalize(position=position)

        # Wait for the user to cancel us.
        if not loop_once:
            print("Use Ctrl-C to quit")

        try:
            sliders_context = self._sliders.GetMyContextFromRoot(self._context)
            while True:
                time.sleep(1 / 32.0)
                q = self._sliders.get_output_port().Eval(sliders_context)
                self._plant.SetPositions(self._plant_context, q)
                self._diagram.Publish(self._context)
                if loop_once:
                    return
        except KeyboardInterrupt:
            pass

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
          frame_id: A geometry::frame_id registered with scene_graph.
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


def main():
    # TODO(todd.rowell) Move this file docstring elsewhere when this gets
    # moved into pydrake.
    args_parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)

    args_parser.add_argument(
        "filename", type=str, default=None,
        help="Path to an SDFormat or URDF file.")
    args_parser.add_argument(
        "--find_resource", action="store_true",
        help="Use FindResourceOrThrow to resolve the filename to a Drake "
             "resource. Use this if the supporting data files are a generated "
             "by Bazel (e.g. the OBJs or PNGs are in @models_internal).")
    args_parser.add_argument(
        "--package_path", type=str, default=None,
        help="Full path to the root package for reading in SDFormat "
             "resources.")

    args_parser.add_argument(
        "-w", "--open-window", dest="browser_new",
        action="store_const", const=1, default=None,
        help="Open the MeshCat display in a new browser window.")
    args_parser.add_argument(
        "--pyplot", action="store_true",
        help="Opens a pyplot figure for rendering using "
             "PlanarSceneGraphVisualizer.")
    # TODO(russt): Consider supporting the PlanarSceneGraphVisualizer
    #  options as additional arguments.
    args_parser.add_argument(
        "--visualize_frames",
        action="store_true",
        help="Visualize the frames as triads for all links.",
    )

    # Create a ModelVisualizer so we can access the defaults.
    defaults = ModelVisualizer()
    args_parser.add_argument(
        "--triad_length",
        type=float,
        dest="triad_length",
        default=defaults.triad_length,
        help="Triad length for frame visualization.",
    )
    args_parser.add_argument(
        "--triad_radius",
        type=float,
        dest="triad_radius",
        default=defaults.triad_radius,
        help="Triad radius for frame visualization.",
    )
    args_parser.add_argument(
        "--triad_opacity",
        type=float,
        dest="triad_opacity",
        default=defaults.triad_opacity,
        help="Triad opacity for frame visualization.",
    )
    args_parser.add_argument(
        "-q", "--position", dest="position",
        type=float, nargs="+", default=[],
        help="A list of positions which must be the same length as the number "
             "of positions in the sdf models.  Note that most models have a "
             "floating-base joint by default (unless the sdf explicitly welds "
             "the base to the world, and so have 7 positions corresponding to "
             "the quaternion representation of that floating-base position).")

    args_parser.add_argument(
        "--loop_once", action='store_true',
        help="Run the evaluation loop once and then quit.")
    args = args_parser.parse_args()

    visualizer = ModelVisualizer(visualize_frames=args.visualize_frames,
                                 triad_length=args.triad_length,
                                 triad_radius=args.triad_radius,
                                 triad_opacity=args.triad_opacity,
                                 browser_new=args.browser_new,
                                 pyplot=args.pyplot)

    if args.package_path:
        # Verify that package.xml is found in the designated path.
        package_path = os.path.abspath(args.package_path)
        package_xml = os.path.join(package_path, "package.xml")
        if not os.path.isfile(package_xml):
            args_parser.error(f"package.xml not found at: {package_path}")

        # Get the package map and populate it using the package path.
        visualizer.parser.package_map().AddPackageXml(package_xml)

    if args.find_resource:
        res_path = os.path.normpath(args.filename)
        print(f"Using FindResourceOrThrow('{res_path}')")
        filename = FindResourceOrThrow(res_path)
    else:
        filename = os.path.abspath(args.filename)
        if not os.path.isfile(filename):
            args_parser.error(f"File does not exist: {filename}")

    visualizer.AddModels(filename)
    visualizer.Run(position=args.position, loop_once=args.loop_once)


if __name__ == '__main__':
    main()
