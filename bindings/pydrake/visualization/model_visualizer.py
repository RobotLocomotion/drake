r"""
Loads a model file (*.sdf or *.urdf) and then displays all models
found in all available visualizers (MeshCat, meldis/drake visualizer).

Joint sliders are available in visualizers that support them by clicking
on "Open Controls" in top right corner.

To have the MeshCat server automatically open in your browser, supply the
`--open-window` flag.

To build all necessary targets and see available command-line options:

    cd drake
    bazel build //tools:drake_visualizer  //tools:model_visualizer

    bazel run //tools:model_visualizer --help

Example usage:

    # Terminal 1
    ./bazel-bin/tools/drake_visualizer

    # Terminal 2 (wait for visualizers to start)
    ./bazel-bin/bindings/pydrake/visualization/model_visualizer \
        --open-window \
        --position 1 0 0 0 0 0 0 0 1 0 0 0 0 0 \
        --find_resource \
        drake/manipulation/models/iiwa_description/iiwa7/iiwa7_with_box_collision.sdf

If your model has all of its data available in your source tree, then you
can remove the need for `--find_resource`:

    ./bazel-bin/bindings/pydrake/visualization/model_visualizer \
        --open-window \
        ${PWD}/manipulation/models/iiwa_description/iiwa7/iiwa7_with_box_collision.sdf

If the model uses package path
(e.g. "package://package_name/model_sdf.obj") to refer to mesh files, you
must also provide the argument `--package_path`:
    ./bazel-bin/bindings/pydrake/visualization/model_visualizer \
        --package_path multibody/parsing/test/box_package \
        multibody/parsing/test/box_package/sdfs/box.sdf

Note:
    The output of running ``show_model`` will include the URL of the
MeshCat server.

Note:
    If you use `bazel run` without `--find_resource`, you are highly
encouraged to use absolute paths, as certain models may not be
prerequisites of this binary.
"""

import argparse
import os

from pydrake.common import FindResourceOrThrow
from pydrake.visualization._model_visualizer import \
    ModelVisualizer as _ModelVisualizer


def main():
    args_parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)

    args_parser.add_argument(
        "filename", type=str, default=None,
        help="Path to an SDFormat or URDF file.")
    args_parser.add_argument(
        "--find_resource", action="store_true",
        help="Use FindResourceOrThrow to resolve the filename to a Drake "
             "resource. Use this if the supporting data files are generated "
             "by Bazel (e.g. the OBJs or PNGs are in @models_internal).")
    args_parser.add_argument(
        "--package_path", type=str, default=None,
        help="Full path to the root package for reading in SDFormat "
             "resources.")

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

    # Create a ModelVisualizer so we can access the defaults.
    defaults = _ModelVisualizer()
    args_parser.add_argument(
        "--triad_length",
        type=float,
        dest="triad_length",
        default=defaults.triad_length,
        help="Triad length for frame visualization.",
    )
    args_parser.add_argument(
        "--triad_radius",
        type=float,
        dest="triad_radius",
        default=defaults.triad_radius,
        help="Triad radius for frame visualization.",
    )
    args_parser.add_argument(
        "--triad_opacity",
        type=float,
        dest="triad_opacity",
        default=defaults.triad_opacity,
        help="Triad opacity for frame visualization.",
    )
    args_parser.add_argument(
        "-q", "--position", dest="position",
        type=float, nargs="+", default=[],
        help="A list of positions which must be the same length as the number "
             "of positions in the sdf models.  Note that most models have a "
             "floating-base joint by default (unless the sdf explicitly welds "
             "the base to the world, and so have 7 positions corresponding to "
             "the quaternion representation of that floating-base position).")

    args_parser.add_argument(
        "--loop_once", action='store_true',
        help="Run the evaluation loop once and then quit.")
    args = args_parser.parse_args()

    visualizer = _ModelVisualizer(visualize_frames=args.visualize_frames,
                                  triad_length=args.triad_length,
                                  triad_radius=args.triad_radius,
                                  triad_opacity=args.triad_opacity,
                                  browser_new=args.browser_new,
                                  pyplot=args.pyplot)

    if args.package_path:
        # Verify that package.xml is found in the designated path.
        package_path = os.path.abspath(args.package_path)
        package_xml = os.path.join(package_path, "package.xml")
        if not os.path.isfile(package_xml):
            args_parser.error(f"package.xml not found at: {package_path}")

        # Get the package map and populate it using the package path.
        visualizer.parser.package_map().AddPackageXml(package_xml)

    if args.find_resource:
        res_path = os.path.normpath(args.filename)
        print(f"Using FindResourceOrThrow('{res_path}')")
        filename = FindResourceOrThrow(res_path)
    else:
        filename = os.path.abspath(args.filename)
        if not os.path.isfile(filename):
            args_parser.error(f"File does not exist: {filename}")

    visualizer.AddModels(filename)
    visualizer.Run(position=args.position, loop_once=args.loop_once)


if __name__ == '__main__':
    main()
