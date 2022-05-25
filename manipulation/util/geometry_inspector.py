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
import os
import webbrowser

import numpy as np

from drake.manipulation.util.show_model import add_triad
from pydrake.common import FindResourceOrThrow
from pydrake.geometry import (
    DrakeVisualizer, MakePhongIllustrationProperties, Meshcat,
    MeshcatVisualizerCpp, SceneGraph)
from pydrake.multibody.parsing import Parser
from pydrake.multibody.meshcat import JointSliders
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.planar_scenegraph_visualizer import \
    ConnectPlanarSceneGraphVisualizer
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
    args_parser.add_argument(
        "-w", "--open-window", dest="browser_new",
        action="store_const", const=1, default=None,
        help="Open the MeshCat display in a new browser window.")

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
    args = args_parser.parse_args()

    if args.find_resource:
        res_path = os.path.normpath(args.filename)
        print(f"Using FindResourceOrThrow('{res_path}')")
        filename = FindResourceOrThrow(res_path)
    else:
        filename = os.path.abspath(args.filename)
        if not os.path.isfile(filename):
            args_parser.error(f"File does not exist: {filename}")

    # NOTE: the meshcat instance is always created in order to create the
    # JointSliders, it will not be displayed unless --show-window is given.
    meshcat = Meshcat()
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())

    # Construct a MultibodyPlant.
    # N.B. Do not use AddMultibodyPlantSceneGraph because we want to inject our
    # custom pose-bundle adjustments for the sliders.
    plant = MultibodyPlant(time_step=0.0)
    plant.RegisterAsSourceForSceneGraph(scene_graph)

    # Add the model from the file and finalize the plant.
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
    parser.AddModelFromFile(filename)

    # Update visualizations.
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
        # TODO(svenevs): how do we use ContactVisualizer?

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

    plant.Finalize()

    # Add sliders to set positions of the joints.
    sliders = builder.AddSystem(JointSliders(meshcat=meshcat, plant=plant))
    to_pose = builder.AddSystem(MultibodyPositionToGeometryPose(plant))
    builder.Connect(sliders.get_output_port(0), to_pose.get_input_port())
    builder.Connect(
        to_pose.get_output_port(),
        scene_graph.get_source_pose_port(plant.get_source_id()))

    # Connect this to drake_visualizer.
    DrakeVisualizer.AddToBuilder(builder=builder, scene_graph=scene_graph)

    # TODO(svenevs): meshcat and hydroelastic usage?  Previously
    # ConnectMeshcatVisualizer(..., prefer_hydro=args.meshcat_hydroelastic).
    # Add a --hydroelastic argument?  The bindings seem to hide this though.
    # TODO(svenevs): I cannot ctrl+C this program to end it if I --open-window,
    # I have to find the PID and kill it *OR* close the meshcat browser window.
    # Why?  Is this OK?  This is not how joint_teleop behaves.
    meshcat_visualizer = MeshcatVisualizerCpp.AddToBuilder(
        builder=builder,
        scene_graph=scene_graph,
        meshcat=meshcat)
    if args.browser_new is not None:
        url = meshcat.web_url()
        webbrowser.open(url=url, new=args.browser_new)

    # Connect to PyPlot.
    if args.pyplot:
        # TODO(svenevs): should we meshcat.Set2dRenderMode()?  Should this
        # become a `--planar` argument instead?
        # meshcat.Set2dRenderMode()
        pyplot = ConnectPlanarSceneGraphVisualizer(builder, scene_graph)

    if len(args.position):
        sliders.SetPositions(args.position)
    # TODO(svenevs): do we want to create this method for JointSliders?
    # elif len(args.joint_position):
    #     sliders.set_joint_position(args.joint_position)

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
