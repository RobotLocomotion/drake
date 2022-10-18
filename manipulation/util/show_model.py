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
    Visualizes models from a file or string buffer in Drake Viz, meldis,
    or MeshCat.

    To use this class to visualize a model, create an instance with
    any desired options, add a model, and then call the instance:

        visualizer = ModelVisualizer(browser_new=True)
        visualizer.add_model_from_string(
            buffer_containing_model, 'sdf', "sample")
        visualizer()

    This class understands the following option keys:
      filename: a path to an SDF or URDF file
      find_resource: a flag that indicates that FindResourceOrThrow will
        be used to resolved the filename to a Drake resource.
      package_path: a full path to the root package for reading SDF
        resources.

      browser_new: a flag that will open the MeshCat display in a new
        browser window.
      pyplot: a flag that will open a pyplot figure for rendering using
        PlanarSceneGraphVisualizer.

      loop_once: a flag that exits the evaluation loop after one pass.

      position: a list of positions for an SDF model; must match the number
        of positions in the model, including 7 positions corresponding to
        the model base if the model base isn't welded to the world.

      visualize_frames: a flag that visualizes the frames as triads for all
        links.
      triad_length: an option length for visualization triads.
      triad_radius: an option radius for visualization triads.
      triad_opacity: an option opacity for visualization triads.

      publish_contacts: an optional flag for VisualizationConfig.
    """

    OPTION_DEFAULTS = {'publish_contacts': True,
                       'triad_length': 0.5,
                       'triad_radius': 0.01,
                       'triad_opacity': 1}

    def __init__(self, **kwargs):
        # Initialize options with our defaults in case we're not initialized
        # using argparse output.
        self.options = dict(**self.OPTION_DEFAULTS)
        if kwargs:
            self.set_options(**kwargs)

        self.builder = None
        self.plant = None
        self.scene_graph = None
        self.meshcat = None
        self.diagram = None
        self.context = None
        self.sliders = None
        self.plant_context = None

    def set_options(self, **kwargs):
        """
        Sets one or more options after initialization.
        Can be called multiple times.
        """
        for key, value in kwargs.items():
            self.options[key] = value

    def add_models_from_file(self, filename, parser_factory=None):
        """
        Adds all models found in an input file.

        In order to run the model, either this method or
        `add_models_from_string` must be called, but not both.

        Args:
          filename: a string containing a model.
          parser_factory: a function which returns a
            `pydrake.multibody.parsing.Parser`.

        See `pydrake.multibody.parsing.Parser.AddAllModelsFromFile`
        for more information on the `filename` argument.
        """
        assert self.builder is None, "Can only add models once."

        # Construct the Plant & SceneGraph.
        self.builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            self.builder, time_step=0.0)

        if not parser_factory:
            def factory(plant):
                return Parser(plant)

            parser_factory = factory

        # Load the model(s) from specified file.
        parser_factory(self.plant).AddAllModelsFromFile(filename)

    def add_models_from_string(self, file_contents, file_type):
        """
        Adds all models found in a string buffer.

        In order to run the model, either this method or
        `add_models_from_file` must be called, but not both.

        Args:
          file_contents: the model data to be parsed.
          file_type: the file type of file_contents.

        See `pydrake.multibody.parsing.Parser.AddModelsFromString`
        for more information on the `file_contents` and `file_type` arguments.
        """
        assert self.builder is None, "Can only add models once."

        self.builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            self.builder, time_step=0.0)

        Parser(self.plant).AddModelsFromString(file_contents, file_type)

    def __call__(self, **kwargs):
        """Sets up visualization and then runs the loop."""
        self.apply_visualization()
        self.run()

    def apply_visualization(self):
        """
        Sets up visualization using set options once a model has been added.
        """
        assert self.builder is not None, "Models have not been added."

        if self.options.get('visualize_frames'):
            # Visualize frames
            # Find all the frames and draw them using add_triad().
            # The frames are drawn using the parsed length.
            # The world frame is drawn thicker than the rest.
            inspector = self.scene_graph.model_inspector()
            for frame_id in inspector.GetAllFrameIds():
                radius = self.options['triad_radius'] * (
                    3 if frame_id == self.scene_graph.world_frame_id() else 1
                    )
                self.add_triad(
                    frame_id,
                    length=self.options['triad_length'],
                    radius=radius,
                    opacity=self.options['triad_opacity'],
                )

        self.plant.Finalize()

        # Connect this to drake_visualizer or meldis. Meldis provides
        # simultaneous visualization of illustration and proximity geometry.
        ApplyVisualizationConfig(
            config=VisualizationConfig(
                publish_contacts=self.options.get('publish_contacts')),
            plant=self.plant,
            scene_graph=self.scene_graph,
            builder=self.builder)

        # Connect to MeshCat: this instance is required to connect
        # e.g., JointSliders.
        self.meshcat = Meshcat()
        # Add two visualizers, one to publish the "illustration" geometry, and
        # another to publish the "collision" geometry.
        MeshcatVisualizer.AddToBuilder(
            self.builder, self.scene_graph, self.meshcat,
            MeshcatVisualizerParams(role=Role.kIllustration, prefix="visual"))
        MeshcatVisualizer.AddToBuilder(
            self.builder, self.scene_graph, self.meshcat,
            MeshcatVisualizerParams(role=Role.kProximity, prefix="collision"))
        sliders = self.builder.AddNamedSystem(
            "joint_sliders", JointSliders(meshcat=self.meshcat,
                                          plant=self.plant))

        if self.options.get('browser_new'):
            url = self.meshcat.web_url()
            webbrowser.open(url=url, new=self.options['browser_new'])

        # Connect to PyPlot.
        if self.options.get('pyplot'):
            ConnectPlanarSceneGraphVisualizer(self.builder, self.scene_graph)

        self.diagram = self.builder.Build()
        self.context = self.diagram.CreateDefaultContext()

        self.sliders = self.diagram.GetSubsystemByName("joint_sliders")
        self.plant_context = self.plant.GetMyContextFromRoot(self.context)

        if self.options.get('position'):
            self.plant.SetPositions(self.plant_context,
                                    self.options['position'])
            self.sliders.SetPositions(self.options['position'])

        # Use Simulator to dispatch initialization events.
        # TODO(eric.cousineau): Simplify as part of #13776 (was #10015).
        Simulator(self.diagram).Initialize()
        # Publish draw messages with current state.
        self.diagram.Publish(self.context)

        # Disable the collision geometry at the start; it can be enabled by the
        # checkbox in the meshcat controls.
        self.meshcat.SetProperty("collision", "visible", False)

    def run(self):
        """
        Runs the model.

        Will iterate once and exit if `loop_once` is set, otherwise will
        loop until the user quits.
        """
        # TODO(todd.rowell) Consider a more accurate name. 'loop'?
        # Wait for the user to cancel us.
        if not self.options.get('loop_once'):
            print("Use Ctrl-C to quit")

        try:
            sliders_context = self.sliders.GetMyContextFromRoot(self.context)
            while True:
                time.sleep(1 / 32.0)
                q = self.sliders.get_output_port().Eval(sliders_context)
                self.plant.SetPositions(self.plant_context, q)
                self.diagram.Publish(self.context)
                if self.options.get('loop_once'):
                    return
        except KeyboardInterrupt:
            pass

    def add_triad(
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
        self.scene_graph.RegisterGeometry(
            self.plant.get_source_id(), frame_id, geom)

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
        self.scene_graph.RegisterGeometry(
            self.plant.get_source_id(), frame_id, geom)

        # z-axis
        X_TG = RigidTransform([0, 0, length / 2.0])
        geom = GeometryInstance(
            X_FT.multiply(X_TG), Cylinder(radius, length), name + " z-axis"
        )
        geom.set_illustration_properties(
            MakePhongIllustrationProperties([0, 0, 1, opacity])
        )
        self.scene_graph.RegisterGeometry(
            self.plant.get_source_id(), frame_id, geom)


def add_filename_and_parser_argparse_arguments(args_parser):
    """
    Adds argparse arguments for filename and model parsing.

    Args:
      args_parser: the `argparse.ArgumentParser` to add arguments to.
    """
    args_parser.add_argument(
        "filename", type=str, default=None,
        help="Path to an SDF or URDF file.")
    args_parser.add_argument(
        "--find_resource", action="store_true",
        help="Use FindResourceOrThrow to resolve the filename to a Drake "
             "resource. Use this if the supporting data files are a generated "
             "by Bazel (e.g. the OBJs or PNGs are in @models_internal).")
    args_parser.add_argument(
        "--package_path", type=str, default=None,
        help="Full path to the root package for reading in SDF resources.")


def parse_filename_and_parser(args_parser, args):
    """
    Parses results from argparse arguments added by
    `add_filename_and_parser_argparse_arguments`:
    * Parses filename (and possibly --find_resource) and returns filename.
    * Parses --package_path and returns make_parser factory.

    Args:
      args_parser: the `argparse.ArgumentParser`.
      args: the namespace object returned by `args_parser.parse_args()`.

    Returns:
      A tuple of (filename, make_parser factory).
    """

    def make_parser(plant):
        parser = Parser(plant)
        # Get the package pathname.
        if args.package_path:
            # Verify that package.xml is found in the designated path.
            package_path = os.path.abspath(args.package_path)
            package_xml = os.path.join(package_path, "package.xml")
            if not os.path.isfile(package_xml):
                args_parser.error(f"package.xml not found at: {package_path}")

            # Get the package map and populate it using the package path.
            parser.package_map().AddPackageXml(package_xml)
        return parser

    if args.find_resource:
        res_path = os.path.normpath(args.filename)
        print(f"Using FindResourceOrThrow('{res_path}')")
        filename = FindResourceOrThrow(res_path)
    else:
        filename = os.path.abspath(args.filename)
        if not os.path.isfile(filename):
            args_parser.error(f"File does not exist: {filename}")

    return filename, make_parser


def add_visualizers_argparse_arguments(args_parser):
    """
    Adds argparse arguments for visualizers.

    Args:
      args_parser: the `argparse.ArgumentParser` to add arguments to.
    """
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
    args_parser.add_argument(
        "--triad_length",
        type=float,
        dest="triad_length",
        default=ModelVisualizer.OPTION_DEFAULTS['triad_length'],
        help="Triad length for frame visualization.",
    )
    args_parser.add_argument(
        "--triad_radius",
        type=float,
        dest="triad_radius",
        default=ModelVisualizer.OPTION_DEFAULTS['triad_radius'],
        help="Triad radius for frame visualization.",
    )
    args_parser.add_argument(
        "--triad_opacity",
        type=float,
        dest="triad_opacity",
        default=ModelVisualizer.OPTION_DEFAULTS['triad_opacity'],
        help="Triad opacity for frame visualization.",
    )
    args_parser.add_argument(
        "--position", type=float, nargs="+", default=[],
        help="A list of positions which must be the same length as the number "
             "of positions in the sdf model.  Note that most models have a "
             "floating-base joint by default (unless the sdf explicitly welds "
             "the base to the world, and so have 7 positions corresponding to "
             "the quaternion representation of that floating-base position).")


def main():
    # TODO(todd.rowell) Move this file docstring elsewhere when this gets
    # moved into pydrake.
    args_parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    add_filename_and_parser_argparse_arguments(args_parser)
    add_visualizers_argparse_arguments(args_parser)
    args_parser.add_argument(
        "--loop_once", action='store_true',
        help="Run the evaluation loop once and then quit.")
    args = args_parser.parse_args()

    show_models = ModelVisualizer(**vars(args))
    filename, make_parser = parse_filename_and_parser(args_parser, args)
    show_models.add_models_from_file(filename, make_parser)
    show_models()


if __name__ == '__main__':
    main()
