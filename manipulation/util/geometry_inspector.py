r"""
Simple tool that parses an SDF or URDF file from the command line and runs a
simple system which takes joint positions from a JointSlider gui and publishes
the resulting geometry poses to drake_visualizer.

If you wish to simply load a model and show it in multiple visualizers, see
`show_model`.

Example usages (each on one line):
bazel run //bindings/pydrake/multibody:geometry_inspector -- \
  $HOME/drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf

bazel run geometry_inspector -- \
  $HOME/drake/multibody/benchmarks/acrobot/acrobot.sdf --position 0.1 0.2

bazel run geometry_inspector -- \
  $HOME/drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf\
  --package_path=$HOME/drake/manipulation/models/iiwa_description
"""

import argparse
import os
import sys

import numpy as np

from pydrake.geometry import ConnectDrakeVisualizer, SceneGraph
from pydrake.manipulation.simple_ui import JointSliders
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.multibody.parsing import PackageMap
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.rendering import MultibodyPositionToGeometryPose


def main():
    args_parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    args_parser.add_argument(
        "filename", type=str,
        help="Path to an SDF or URDF file.")
    args_parser.add_argument(
        "--package_path",
        type=str,
        help="Full path to the root package for reading in SDF resources.",
        default=[])
    position_group = args_parser.add_mutually_exclusive_group()
    position_group.add_argument(
        "--position", type=float, nargs="+", default=[],
        help="A list of positions which must be the same length as the number "
             "of positions in the sdf model.  Note that most models have a "
             "floating-base joint by default (unless the sdf explicitly welds "
             "the base to the world, and so have 7 positions corresponding to "
             "the quaternion representation of that floating-base position.")
    position_group.add_argument(
        "--joint_position", type=float, nargs="+", default=[],
        help="A list of positions which must be the same length as the number "
             "of positions ASSOCIATED WITH JOINTS in the sdf model.  This "
             "does not include, e.g., floating-base coordinates, which will "
             "be assigned a default value.")
    args_parser.add_argument(
        "--test", action='store_true',
        help="Disable opening the gui window for testing.")
    # TODO(russt): Add option to weld the base to the world pending the
    # availability of GetUniqueBaseBody requested in #9747.
    args = args_parser.parse_args()
    filename = args.filename
    if not os.path.isfile(filename):
        args_parser.error("File does not exist: {}".format(filename))

    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())

    # Construct a MultibodyPlant.
    plant = MultibodyPlant()
    plant.RegisterAsSourceForSceneGraph(scene_graph)

    # Create the parser
    parser = Parser(plant)

    # Get the package pathname.
    if len(args.package_path):
        # Verify that package.xml is found in the designated path.
        package_path = os.path.abspath(args.package_path)
        if not os.path.isfile(os.path.join(package_path, "package.xml")):
            parser.error("package.xml not found at: {}".format(package_path))

        # Get the package map and populate it using the package path.
        package_map = parser.mutable_package_map()
        package_map.PopulateFromFolder(package_path)

    # Add the model from the file and finalize the plant.
    parser.AddModelFromFile(args.filename)
    plant.Finalize(scene_graph)

    # Add sliders to set positions of the joints.
    sliders = builder.AddSystem(JointSliders(robot=plant))
    to_pose = builder.AddSystem(MultibodyPositionToGeometryPose(plant))
    builder.Connect(sliders.get_output_port(0), to_pose.get_input_port())
    builder.Connect(
        to_pose.get_output_port(),
        scene_graph.get_source_pose_port(plant.get_source_id()))

    # Connect this to drake_visualizer.
    ConnectDrakeVisualizer(builder=builder, scene_graph=scene_graph)

    if len(args.position):
        sliders.set_position(args.position)
    elif len(args.joint_position):
        sliders.set_joint_position(args.joint_position)

    # Make the diagram and run it.
    diagram = builder.Build()
    simulator = Simulator(diagram)

    simulator.set_publish_every_time_step(False)

    if args.test:
        sliders.window.withdraw()
        simulator.StepTo(0.1)
    else:
        simulator.set_target_realtime_rate(1.0)
        simulator.StepTo(np.inf)


if __name__ == '__main__':
    main()
