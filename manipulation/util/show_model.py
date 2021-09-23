r"""Loads a model file (*.sdf or *.urdf) and then displays it in all
available visualizers.

To view the model with a simple gui to change joint positions, please see
`geometry_inspector`.

To build all necessary targets and see available command-line options:

    cd drake
    bazel build \
        //tools:drake_visualizer @meshcat_python//:meshcat-server \
        //manipulation/util:show_model

    ./bazel-bin/manipulation/util/show_model --help

Example usage:

    # Terminal 1
    ./bazel-bin/tools/drake_visualizer
    # Terminal 2
    ./bazel-bin/external/meshcat_python/meshcat-server
    # Terminal 3 (wait for visualizers to start)
    ./bazel-bin/manipulation/util/show_model \
        --meshcat default \
        --find_resource \
        drake/manipulation/models/iiwa_description/iiwa7/iiwa7_no_collision.sdf

If your model has all of its data available in your source tree, then you can
remove the need for `--find_resource`:

    ./bazel-bin/manipulation/util/show_model \
        --meshcat default \
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

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import MakePhongIllustrationProperties
from pydrake.geometry import DrakeVisualizer
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.meshcat_visualizer import (
    ConnectMeshcatVisualizer, MeshcatVisualizer)
from pydrake.systems.planar_scenegraph_visualizer import (
    ConnectPlanarSceneGraphVisualizer,
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
             "by Bazel (e.g. the OBJs or PNGs are in @models).")
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


def add_visualizers_argparse_arguments(args_parser):
    """
    Adds argparse arguments for visualizers.
    """
    MeshcatVisualizer.add_argparse_argument(args_parser)
    args_parser.add_argument(
        "--pyplot", action="store_true",
        help="Opens a pyplot figure for rendering using "
             "PlanarSceneGraphVisualizer.")
    # TODO(russt): Consider supporting the PlanarSceneGraphVisualizer
    #  options as additional arguments.
    args_parser.add_argument(
        "--visualize_collisions", action="store_true",
        help="Visualize the collision geometry in the visualizer. The "
        "collision geometries will be shown in red to differentiate "
        "them from the visual geometries.")


def parse_visualizers(args_parser, args):
    """
    Parses argparse arguments for visualizers, returning update_visualization
    and connect_visualizers.
    """

    def update_visualization(plant, scene_graph):
        if args.visualize_collisions:
            # Find all the geometries that have not already been marked as
            # 'illustration' (visual) and assume they are collision geometries.
            # Then add illustration properties to them that will draw them in
            # red and fifty percent translucent.
            source_id = plant.get_source_id()
            inspector = scene_graph.model_inspector()
            diffuse_rgba = [1, 0, 0, 0.5]
            red_illustration = MakePhongIllustrationProperties(diffuse_rgba)
            for geometry_id in inspector.GetAllGeometryIds():
                if inspector.GetIllustrationProperties(geometry_id) is None:
                    scene_graph.AssignRole(
                        source_id, geometry_id, red_illustration)

    def connect_visualizers(builder, plant, scene_graph):
        # Connect this to drake_visualizer.
        DrakeVisualizer.AddToBuilder(builder=builder, scene_graph=scene_graph)

        # Connect to Meshcat.
        if args.meshcat is not None:
            meshcat_viz = ConnectMeshcatVisualizer(
                builder, scene_graph, zmq_url=args.meshcat,
                role=args.meshcat_role,
                prefer_hydro=args.meshcat_hydroelastic)

        # Connect to PyPlot.
        if args.pyplot:
            pyplot = ConnectPlanarSceneGraphVisualizer(builder, scene_graph)

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
    # Load the model file.
    make_parser(plant).AddModelFromFile(filename)
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


if __name__ == '__main__':
    main()
