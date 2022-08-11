r"""Loads a model file (*.sdf or *.urdf) and then displays it in all
available visualizers.

To view the model with a simple gui to change joint positions, please see
`geometry_inspector`.

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
        --meshcat \
        --open-window \
        --find_resource \
        drake/manipulation/models/iiwa_description/iiwa7/iiwa7_no_collision.sdf

If your model has all of its data available in your source tree, then you can
remove the need for `--find_resource`:

    ./bazel-bin/manipulation/util/show_model \
        --meshcat \
        --open-window \
        ${PWD}/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf

If the model uses package path (e.g. "package://package_name/model_sdf.obj") to
refer mesh files, you also have to provide the argument `--package_path`:
    ./bazel-bin/manipulation/util/show_model \
        --package_path \
        manipulation/models/iiwa_description \
        ./manipulation/models/iiwa_description/iiwa7/iiwa7_no_collision.sdf

Note:
    If `--meshcat` is not specified, no meshcat visualization will take
place.

Note:
    If you use `bazel run` without `--find_resource`, it is highly encouraged
that you use absolute paths, as certain models may not be prerequisites of this
binary.
"""

import argparse
import os
import time
import warnings
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
    Rgba,
    Role,
)
from pydrake.math import RigidTransform, RotationMatrix
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


def add_filename_and_parser_argparse_arguments(args_parser):
    """
    Adds argparse arguments for filename and model parsing.
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
    Parses results from arguments added by
    `add_filename_and_parser_argparse_arguments`:
    * Parses filename (and possibly --find_resource) and returns filename.
    * Parses --package_path and returns make_parser factory.
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


class _StringToRoleAction(argparse.Action):
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
            raise ValueError(f"Role parameter got invalid value {values}")

        setattr(namespace, self.dest, mapped_value)


def add_visualizers_argparse_arguments(args_parser):
    """
    Adds argparse arguments for visualizers.
    """
    args_parser.add_argument(
        "--meshcat", action="store_true",
        help="Enable the MeshCat display.")
    args_parser.add_argument(
        "-w", "--open-window", dest="browser_new",
        action="store_const", const=1, default=None,
        help="Open the MeshCat display in a new browser window.")
    args_parser.add_argument(
        "--meshcat_role", action=_StringToRoleAction,
        default=Role.kIllustration, choices=['illustration', 'proximity'],
        help="Defines the role of the geometry to visualize")
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
        default=0.5,
        help="Triad length for frame visualization.",
    )
    args_parser.add_argument(
        "--triad_radius",
        type=float,
        dest="triad_radius",
        default=0.01,
        help="Triad radius for frame visualization.",
    )
    args_parser.add_argument(
        "--triad_opacity",
        type=float,
        dest="triad_opacity",
        default=1,
        help="Triad opacity for frame visualization.",
    )


def add_triad(
    source_id,
    frame_id,
    scene_graph,
    *,
    length,
    radius,
    opacity,
    X_FT=RigidTransform(),
    name="frame",
):
    """
    Adds illustration geometry representing the coordinate frame, with the
    x-axis drawn in red, the y-axis in green and the z-axis in blue. The axes
    point in +x, +y and +z directions, respectively.
    Based on [code permalink](https://github.com/RussTedrake/manipulation/blob/5e59811/manipulation/scenarios.py#L367-L414).# noqa
    Args:
    source_id: The source registered with SceneGraph.
    frame_id: A geometry::frame_id registered with scene_graph.
    scene_graph: The SceneGraph with which we will register the geometry.
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
    scene_graph.RegisterGeometry(source_id, frame_id, geom)

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
    scene_graph.RegisterGeometry(source_id, frame_id, geom)

    # z-axis
    X_TG = RigidTransform([0, 0, length / 2.0])
    geom = GeometryInstance(
        X_FT.multiply(X_TG), Cylinder(radius, length), name + " z-axis"
    )
    geom.set_illustration_properties(
        MakePhongIllustrationProperties([0, 0, 1, opacity])
    )
    scene_graph.RegisterGeometry(source_id, frame_id, geom)


def parse_visualizers(args_parser, args):
    """
    Parses argparse arguments for visualizers, returning update_visualization,
    and connect_visualizers.

    When ``args.meshcat`` is ``True``, ``connect_visualizers`` will return the
    underlying ``Meshcat`` instance.  Otherwise, it returns ``None``.
    """
    def update_visualization(plant, scene_graph):
        if args.visualize_frames:
            # Visualize frames
            # Find all the frames and draw them using add_triad().
            # The frames are drawn using the parsed length.
            # The world frame is drawn thicker than the rest.
            inspector = scene_graph.model_inspector()
            for frame_id in inspector.GetAllFrameIds():
                radius = args.triad_radius * (
                    3 if frame_id == scene_graph.world_frame_id() else 1
                    )
                add_triad(
                    plant.get_source_id(),
                    frame_id,
                    scene_graph,
                    length=args.triad_length,
                    radius=radius,
                    opacity=args.triad_opacity,
                )

    def connect_visualizers(builder, plant, scene_graph,
                            publish_contacts=True):
        # Connect this to drake_visualizer or meldis. Meldis provides
        # simultaneous visualization of illustration and proximity geometry.
        ApplyVisualizationConfig(
            config=VisualizationConfig(publish_contacts=publish_contacts),
            plant=plant,
            scene_graph=scene_graph,
            builder=builder)

        # Connect to Meshcat.  If the consuming application needs to connect,
        # e.g., JointSliders, the meshcat instance is required.
        meshcat = None
        if args.meshcat:
            meshcat = Meshcat()
            meshcat_vis_params = MeshcatVisualizerParams()
            meshcat_vis_params.role = args.meshcat_role
            MeshcatVisualizer.AddToBuilder(
                builder=builder, scene_graph=scene_graph, meshcat=meshcat,
                params=meshcat_vis_params)
            if args.browser_new is not None:
                url = meshcat.web_url()
                webbrowser.open(url=url, new=args.browser_new)
        else:
            if args.browser_new is not None:
                args_parser.error("-w / --open-window require --meshcat")

        # Connect to PyPlot.
        if args.pyplot:
            pyplot = ConnectPlanarSceneGraphVisualizer(builder, scene_graph)

        return meshcat

    return update_visualization, connect_visualizers


def main():
    args_parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    add_filename_and_parser_argparse_arguments(args_parser)
    add_visualizers_argparse_arguments(args_parser)
    args = args_parser.parse_args()
    filename, make_parser = parse_filename_and_parser(args_parser, args)
    update_visualization, connect_visualizers = parse_visualizers(
        args_parser, args)

    # Construct Plant + SceneGraph.
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    # Load the model(s) from specified file.
    make_parser(plant).AddAllModelsFromFile(filename)
    update_visualization(plant, scene_graph)
    plant.Finalize()

    connect_visualizers(builder, plant, scene_graph)
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()

    # Use Simulator to dispatch initialization events.
    # TODO(eric.cousineau): Simplify as part of #10015.
    Simulator(diagram).Initialize()
    # Publish draw messages with current state.
    diagram.Publish(context)

    # Wait for the user to cancel us.
    if args.meshcat:
        print("Use Ctrl-C to quit")
        try:
            time.sleep(1e3)
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
