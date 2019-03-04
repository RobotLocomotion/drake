r"""Loads a model file (*.sdf or *.urdf) and then displays it in all
available visualizers.

To view the model with a simple gui to change joint positions, please see
`geometry_inspector`.

Example usage:

    cd drake
    bazel build \
        //tools:drake_visualizer @meshcat_python//:meshcat-server \
        //manipulation/util:show_model

    # Terminal 1
    ./bazel-bin/tools/drake_visualizer
    # Terminal 2
    ./bazel-bin/external/meshcat_python/meshcat-server
    # Terminal 3 (wait for visualizers to start)
    ./bazel-bin/manipulation/util/show_model \
        --meshcat default \
        ./manipulation/models/iiwa_description/iiwa7/iiwa7_no_collision.sdf

NOTE: If `--meshcat` is not specified, no meshcat visualization will take
place.

NOTE: If you use `bazel run`, it is highly encouraged that you use absolute
paths, as certain models may not be prerequisites of this binary.
"""

from __future__ import print_function
import argparse
import os
import sys

from pydrake.lcm import DrakeLcm
from pydrake.geometry import ConnectDrakeVisualizer, SceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "filename", type=str,
        help="Path to an SDF or URDF file.")
    MeshcatVisualizer.add_argparse_argument(parser)
    args = parser.parse_args()
    filename = args.filename
    if not os.path.isfile(filename):
        parser.error("File does not exist: {}".format(filename))

    # Construct Plant + SceneGraph.
    builder = DiagramBuilder()
    plant = builder.AddSystem(MultibodyPlant())
    scene_graph = builder.AddSystem(SceneGraph())
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    builder.Connect(
        scene_graph.get_query_output_port(),
        plant.get_geometry_query_input_port())
    builder.Connect(
        plant.get_geometry_poses_output_port(),
        scene_graph.get_source_pose_port(plant.get_source_id()))
    # Load the model file.
    Parser(plant).AddModelFromFile(filename)
    plant.Finalize()

    # Publish to Drake Visualizer.
    drake_viz_pub = ConnectDrakeVisualizer(builder, scene_graph)

    # Publish to Meshcat.
    if args.meshcat is not None:
        meshcat_viz = builder.AddSystem(
            MeshcatVisualizer(scene_graph, zmq_url=args.meshcat))
        builder.Connect(
            scene_graph.get_pose_bundle_output_port(),
            meshcat_viz.get_input_port(0))

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()

    # Use Simulator to dispatch initialization events.
    # TODO(eric.cousineau): Simplify as part of #10015.
    Simulator(diagram).Initialize()
    # Publish draw messages with current state.
    diagram.Publish(context)


if __name__ == '__main__':
    main()
