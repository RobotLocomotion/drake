r"""The ``model_visualizer`` program displays a model file (e.g., ``*.sdf``) in
Drake's built-in visualizers (MeshCat and/or Meldis). When viewing in MeshCat,
joint sliders to posture the model are available by clicking on "Open Controls"
in the top right corner.

This command-line module is provided for convenience, but the feature is also
available via the library class ``pydrake.visualization.ModelVisualizer``.

From a Drake source build, run this module as::

  bazel run //tools:model_visualizer -- --help

From a Drake binary release (including pip releases), run this module as::

  python3 -m pydrake.visualization.model_visualizer --help

Refer to the instructions printed by ``--help`` for additional details.

An example of viewing an iiwa model file::

  python3 -m pydrake.visualization.model_visualizer --open-window \
        package://drake/manipulation/models/iiwa_description/iiwa7/iiwa7_with_box_collision.sdf

This program respects the ``ROS_PACKAGE_PATH``; if your model uses external
resources then you will need to set that environment variable.
"""

import argparse
import os

from pydrake.visualization._model_visualizer import \
    ModelVisualizer as _ModelVisualizer


def _main():
    args_parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)

    # Many of our command line arguments map directly onto named arguments to
    # the ModelVisualizer constructor. We'll obey those constructor defaults
    # as we define the argparse defaults.
    defaults = _ModelVisualizer._get_constructor_defaults()

    args_parser.add_argument(
        "filename", type=str, default=None,
        help="Path to an SDFormat or URDF file, or a package:// URL.")

    assert defaults["browser_new"] is False
    args_parser.add_argument(
        "-w", "--open-window", dest="browser_new",
        action="store_true",
        help="Open the MeshCat display in a new browser window.")
    assert defaults["pyplot"] is False
    args_parser.add_argument(
        "--pyplot", action="store_true",
        help="Opens a pyplot figure for rendering using "
             "PlanarSceneGraphVisualizer.")
    # TODO(russt): Consider supporting the PlanarSceneGraphVisualizer
    #  options as additional arguments.
    assert defaults["visualize_frames"] is False
    args_parser.add_argument(
        "--visualize_frames",
        action="store_true",
        help="Visualize the frames as triads for all links.",
    )

    args_parser.add_argument(
        "--triad_length",
        type=float,
        dest="triad_length",
        default=defaults["triad_length"],
        help="Triad length for frame visualization.",
    )
    args_parser.add_argument(
        "--triad_radius",
        type=float,
        dest="triad_radius",
        default=defaults["triad_radius"],
        help="Triad radius for frame visualization.",
    )
    args_parser.add_argument(
        "--triad_opacity",
        type=float,
        dest="triad_opacity",
        default=defaults["triad_opacity"],
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

    package_map = visualizer.parser().package_map()
    package_map.PopulateFromRosPackagePath()

    # Resolve the filename if necessary.
    filename = args.filename
    if filename.startswith("package://"):
        # TODO(jwnimmer-tri) PackageMap should provide a function for this.
        suffix = filename[len("package://"):]
        package, relative_path = suffix.split("/", maxsplit=1)
        filename = os.path.join(package_map.GetPath(package), relative_path)
    filename = os.path.abspath(filename)
    if not os.path.isfile(filename):
        args_parser.error(f"File does not exist: {filename}")

    visualizer.AddModels(filename)
    visualizer.Run(position=args.position, loop_once=args.loop_once)


if __name__ == '__main__':
    _main()
