r"""
Simple tool that parses an SDFormat or URDF file from the command line and runs
a simple system which takes joint positions from a JointSlider gui and
publishes the resulting geometry poses to available visualizers.

If you wish to simply load a model and show it in multiple visualizers, see
`show_model`.

To build all necessary targets and see available command-line options:

    cd drake
    bazel build \
        //tools:drake_visualizer @meshcat_python//:meshcat-server \
        //manipulation/util:geometry_inspector

    ./bazel-bin/manipulation/util/geometry_inspector --help

Example usage (drake visualizer):

    # Terminal 1
    ./bazel-bin/tools/drake_visualizer
    # Terminal 2
    ./bazel-bin/manipulation/util/geometry_inspector \
        --find_resource \
        drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf

If your model has all of its data available in your source tree, then you can
remove the need for `--find_resource`:

    ./bazel-bin/manipulation/util/geometry_inspector \
        ${PWD}/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf

Example usage (meshcat):

    # Terminal 1
    ./bazel-bin/external/meshcat_python/meshcat-server
    # Terminal 2
    ./bazel-bin/manipulation/util/geometry_inspector \
        --meshcat default \
        --find_resource \
        drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf

Optional argument examples:
    ./bazel-bin/manipulation/util/geometry_inspector \
        --position 0.1 0.2 \
        --find_resource drake/multibody/benchmarks/acrobot/acrobot.sdf

Note:
    ``geometry_inspector`` will always send the lcm draw messages to
drake_visualizer (they have no effect unless you open the visualizer).

Note:
    If ``--meshcat`` is not specified, no meshcat visualization will take
place.

Note:
    If you use `bazel run` without `--find_resource`, it is highly encouraged
that you use absolute paths, as certain models may not be prerequisites of this
binary.
"""

import argparse

import numpy as np

from pydrake.geometry import Meshcat, SceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.multibody.meshcat import (
    JointSliders,
)
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.rendering import MultibodyPositionToGeometryPose


def main():
    args_parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
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
    # TODO(eric.cousineau): Support sliders (or widgets) for floating body
    # poses.
    # TODO(russt): Once floating body sliders are supported, add an option to
    # disable them too, either by welding via GetUniqueBaseBody #9747 or by
    # hiding the widgets.
    args_parser.add_argument(
        "--test", action='store_true',
        help="Disable opening the slider gui window for testing.")
    # TODO(russt): Consider supporting the PlanarSceneGraphVisualizer
    #  options as additional arguments.
    args_parser.add_argument(
        "--visualize_collisions", action="store_true",
        help="Visualize the collision geometry in the visualizer. The "
        "collision geometries will be shown in red to differentiate "
        "them from the visual geometries.")
    args = args_parser.parse_args()
    meshcat = Meshcat()
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())

    # Construct a MultibodyPlant.
    # N.B. Do not use AddMultibodyPlantSceneGraph because we want to inject our
    # custom pose-bundle adjustments for the sliders.
    plant = MultibodyPlant(time_step=0.0)
    parser = Parser(plant)
    plant.RegisterAsSourceForSceneGraph(scene_graph)

    # Add the model from the file and finalize the plant.
    parser.AddModelFromFile(args.filename)
    plant.Finalize()

    # Add sliders to set positions of the joints.
    sliders = builder.AddSystem(JointSliders(meshcat=meshcat, plant=plant))
    to_pose = builder.AddSystem(MultibodyPositionToGeometryPose(plant))
    builder.Connect(sliders.get_output_port(0), to_pose.get_input_port())
    builder.Connect(
        to_pose.get_output_port(),
        scene_graph.get_source_pose_port(plant.get_source_id()))

    # Make the diagram and run it.
    diagram = builder.Build()
    simulator = Simulator(diagram)

    if args.test:
        simulator.AdvanceTo(0.1)
    else:
        simulator.set_target_realtime_rate(1.0)
        simulator.AdvanceTo(np.inf)


if __name__ == '__main__':
    main()
