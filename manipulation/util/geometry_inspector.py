r"""
Simple tool that parses an SDF or URDF file from the command line and runs a
simple system which takes joint positions from a JointSlider gui and publishes
the resulting geometry poses to drake_visualizer.

If you wish to simply load a model and show it in multiple visualizers, see
`show_model`.

Examples:

    # Build everything necessary.
    bazel build //tools:drake_visualizer //manipulation/util:geometry_inspector

    # Terminal 1 - Visualizer.
    ./bazel-bin/tools/drake_visualizer

    # Terminal 2 - Show IIWA.
    ./bazel-bin/manipulation/util/geometry_inspector \
        ./manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf
    # - Ctrl+C, then show acrobot.
    ./bazel-bin/manipulation/util/geometry_inspector \
        ./multibody/benchmarks/acrobot/acrobot.sdf --position 0.1 0.2

NOTE: If you use `bazel run`, it is highly encouraged that you use absolute
paths, as certain models may not be prerequisites of this binary.
"""

from __future__ import print_function
import argparse
import os

import numpy as np

from pydrake.geometry import ConnectDrakeVisualizer, SceneGraph
from pydrake.manipulation.simple_ui import JointSliders
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.rendering import MultibodyPositionToGeometryPose


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "filename", type=str,
        help="Path to an SDF or URDF file.")
    position_group = parser.add_mutually_exclusive_group()
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
    parser.add_argument(
        "--test", action='store_true',
        help="Disable opening the gui window for testing.")
    # TODO(russt): Add option to weld the base to the world pending the
    # availability of GetUniqueBaseBody requested in #9747.
    args = parser.parse_args()
    filename = args.filename
    if not os.path.isfile(filename):
        parser.error("File does not exist: {}".format(filename))

    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())

    # Construct a MultibodyPlant and load the SDF into it.
    plant = MultibodyPlant()
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    Parser(plant).AddModelFromFile(filename)
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
