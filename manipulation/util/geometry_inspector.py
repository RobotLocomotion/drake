r"""
Simple tool that parses an SDFormat or URDF file from the command line and runs
a simple system which takes joint positions from a meshcat JointSlider gui and
publishes the resulting geometry poses to available visualizers.  To have the
meshcat server automatically open in your browser, supply the --open-window
flag; the joint sliders will be accessible by clicking on "Open Controls" in
top right corner.

To build all necessary targets and see available command-line options:

    cd drake
    bazel build \
        //tools:drake_visualizer //manipulation/util:geometry_inspector
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

    ./bazel-bin/manipulation/util/geometry_inspector \
        --open-window \
        --find_resource \
        drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf

Example usage (pyplot):

    ./bazel-bin/manipulation/util/geometry_inspector \
        --pyplot \
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
    If ``--open-window`` is not specified, in the output of running
``geometry_inspector`` the URL of the meshcat server is printed.  In order to
perform any manipulations of the robot joints, you must open the meshcat server
in a browser window and click on "Open Controls" in the top right corner in
order to have access to the JointSliders controls.

Note:
    If you use `bazel run` without `--find_resource`, it is highly encouraged
that you use absolute paths, as certain models may not be prerequisites of this
binary.
"""

import argparse

import numpy as np

from pydrake.geometry import SceneGraph
from pydrake.multibody.meshcat import JointSliders
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.rendering import MultibodyPositionToGeometryPose

from drake.manipulation.util.show_model import (
    add_filename_and_parser_argparse_arguments,
    parse_filename_and_parser,
    add_visualizers_argparse_arguments,
    parse_visualizers,
)


def main():
    args_parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    add_filename_and_parser_argparse_arguments(args_parser)
    add_visualizers_argparse_arguments(args_parser)
    args_parser.add_argument(
        "--position", type=float, nargs="+", default=[],
        help="A list of positions which must be the same length as the number "
             "of positions in the sdf model.  Note that most models have a "
             "floating-base joint by default (unless the sdf explicitly welds "
             "the base to the world, and so have 7 positions corresponding to "
             "the quaternion representation of that floating-base position.")
    # TODO(eric.cousineau): Support sliders (or widgets) for floating body
    # poses.
    # TODO(russt): Once floating body sliders are supported, add an option to
    # disable them too, either by welding via GetUniqueBaseBody #9747 or by
    # hiding the widgets.
    args_parser.add_argument(
        "--test", action='store_true',
        help="Disable opening the slider gui window for testing.")
    args = args_parser.parse_args()
    # NOTE: meshcat is required to create the JointSliders.
    args.meshcat = True
    filename, make_parser = parse_filename_and_parser(args_parser, args)
    update_visualization, connect_visualizers = parse_visualizers(
        args_parser, args)

    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())

    # Construct a MultibodyPlant.
    # N.B. Do not use AddMultibodyPlantSceneGraph because we want to inject our
    # custom pose-bundle adjustments for the sliders.
    plant = MultibodyPlant(time_step=0.0)
    plant.RegisterAsSourceForSceneGraph(scene_graph)

    # Add the model from the file and finalize the plant.
    make_parser(plant).AddModelFromFile(filename)
    update_visualization(plant, scene_graph)
    plant.Finalize()

    meshcat = connect_visualizers(builder, plant, scene_graph,
                                  publish_contacts=False)
    assert meshcat is not None, "Meshcat visualizer not created but required."

    # Add sliders to set positions of the joints.
    sliders = builder.AddSystem(JointSliders(meshcat=meshcat, plant=plant))
    to_pose = builder.AddSystem(MultibodyPositionToGeometryPose(plant))
    builder.Connect(sliders.get_output_port(0), to_pose.get_input_port())
    builder.Connect(
        to_pose.get_output_port(),
        scene_graph.get_source_pose_port(plant.get_source_id()))

    if len(args.position):
        sliders.SetPositions(args.position)

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
