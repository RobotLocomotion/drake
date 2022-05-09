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
import numpy as np

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
from pydrake.all import (
    AddMultibodyPlantSceneGraph, Cylinder,
    DiagramBuilder, 
    FindResourceOrThrow, GeometryInstance, 
    MakePhongIllustrationProperties,  Parser, RigidTransform, 
    RotationMatrix)

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
    args_parser.add_argument(
        "--visualize_frames", nargs='?',
        type=float,
        dest='triad_length', const=1,default=0,
        help="Visualize the frames as triads for all links.")

def parse_visualizers(args_parser, args):
    """
    Parses argparse arguments for visualizers, returning update_visualization,
    and connect_visualizers.
    """

    def update_visualization(plant, scene_graph):
        def AddTriad(source_id, frame_id,
            scene_graph, length=.25,
            radius=0.01, opacity=1.,
            X_FT=RigidTransform(), name="frame"):
            """
            Adds illustration geometry representing the coordinate frame, with the
            x-axis drawn in red, the y-axis in green and the z-axis in blue. The axes
            point in +x, +y and +z directions, respectively.
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
            X_TG = RigidTransform(RotationMatrix.MakeYRotation(np.pi / 2),
                                [length / 2., 0, 0])
            geom = GeometryInstance(X_FT.multiply(X_TG), Cylinder(radius, length),
                                    name + " x-axis")
            geom.set_illustration_properties(
                MakePhongIllustrationProperties([1, 0, 0, opacity]))
            scene_graph.RegisterGeometry(source_id, frame_id, geom)

            # y-axis
            X_TG = RigidTransform(RotationMatrix.MakeXRotation(np.pi / 2),
                                [0, length / 2., 0])
            geom = GeometryInstance(X_FT.multiply(X_TG), Cylinder(radius, length),
                                    name + " y-axis")
            geom.set_illustration_properties(
                MakePhongIllustrationProperties([0, 1, 0, opacity]))
            scene_graph.RegisterGeometry(source_id, frame_id, geom)

            # z-axis
            X_TG = RigidTransform([0, 0, length / 2.])
            geom = GeometryInstance(X_FT.multiply(X_TG), Cylinder(radius, length),
                                    name + " z-axis")
            geom.set_illustration_properties(
                MakePhongIllustrationProperties([0, 0, 1, opacity]))
            scene_graph.RegisterGeometry(source_id, frame_id, geom)
    
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
        
        if args.triad_length!=0:
            # Visualize frames 
            # Find all the frames and plots them using AddTriad().
            # The frames are ploted using the parsed length. 
            # The world frame is plotted thicker than the rest. 
            length=args.triad_length
            inspector = scene_graph.model_inspector()
            for i,frame_id in enumerate(inspector.GetAllFrameIds()):
                #The world frame is the last in the list
                if i<len(inspector.GetAllFrameIds())-1:
                    AddTriad(plant.get_source_id(),frame_id,scene_graph,length)      
                else:
                    AddTriad(plant.get_source_id(),frame_id,scene_graph,length, radius=0.03)   

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


if __name__ == '__main__':
    main()
