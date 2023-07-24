r"""The ``model_visualizer`` program displays a model file (e.g., ``*.sdf``) in
Drake's built-in visualizers (MeshCat and/or Meldis). When viewing in MeshCat,
joint sliders to posture the model are available by clicking on "Open Controls"
in the top right corner.

If the loaded model file is changed, it can be reloaded by pressing
the "Reload Model Files" button, which will attempt to maintain slider values
once reloading is finished. To exit, press the "Stop Running" button or press
the Escape key.

This command-line module is provided for convenience, but the feature is also
available via the library class ``pydrake.visualization.ModelVisualizer``.

From a Drake source build, run this module as::

  bazel run //tools:model_visualizer -- --help

From a Drake binary release (including pip releases), run this module as::

  python3 -m pydrake.visualization.model_visualizer --help

For binary releases (except for pip) there is also a shortcut available as::

  /opt/drake/bin/model_visualizer

Refer to the instructions printed by ``--help`` for additional details.

An example of viewing an iiwa model file::

  python3 -m pydrake.visualization.model_visualizer --open-window \
        package://drake/manipulation/models/iiwa_description/iiwa7/iiwa7_with_box_collision.sdf

This program respects the ``ROS_PACKAGE_PATH``; if your model uses external
resources then you will need to set that environment variable.
"""

import argparse
import logging
import os
from pathlib import Path

from pydrake.visualization._model_visualizer import \
    ModelVisualizer as _ModelVisualizer


def _main():
    # Use a few color highlights for the user's terminal output.
    logging.addLevelName(logging.INFO, "\033[36mINFO\033[0m")
    logging.addLevelName(logging.WARNING, "\033[33mWARNING\033[0m")
    logging.addLevelName(logging.ERROR, "\033[31mERROR\033[0m")
    format = "%(levelname)s: %(message)s"
    logging.basicConfig(level=logging.INFO, format=format)

    # Prepare to parse arguments.
    args_parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)

    # Many of our command line arguments map directly onto named arguments to
    # the ModelVisualizer constructor. We'll obey those constructor defaults
    # as we define the argparse defaults.
    defaults = _ModelVisualizer._get_constructor_defaults()

    args_parser.add_argument(
        "filename", nargs="+", type=str,
        help="Filesystem path to an SDFormat, URDF, OBJ, or DMD file; "
        "or a package:// URL to use a ROS package path.")

    assert defaults["browser_new"] is False
    args_parser.add_argument(
        "-w", "--open-window", dest="browser_new",
        action="store_true",
        help="Open the MeshCat display in a new browser window.")
    assert defaults["pyplot"] is False
    args_parser.add_argument(
        "--pyplot", action="store_true",
        help="Open a pyplot figure for rendering using "
             "PlanarSceneGraphVisualizer.")
    # TODO(russt): Consider supporting the PlanarSceneGraphVisualizer
    #  options as additional arguments.
    assert defaults["visualize_frames"] is False
    args_parser.add_argument(
        "--visualize_frames",
        action="store_true",
        help="Visualize the frames as triads for all links.",
    )
    assert defaults["show_rgbd_sensor"] is False
    args_parser.add_argument(
        "--show_rgbd_sensor",
        action="store_true",
        help="Add and show an RgbdSensor. At the moment, the image display "
             "uses a native window so will not work in a remote or cloud "
             "runtime environment.",
    )
    assert defaults["environment_map"] == Path()
    args_parser.add_argument(
        "--environment_map", default=Path(), type=Path,
        help="Filesystem path to an image to be used as an environment map. "
             "It must be an image type normally used by your browser (e.g., "
             ".jpg, .png, etc.). HDR images are not supported yet."
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

    if 'BUILD_WORKSPACE_DIRECTORY' in os.environ:
        os.chdir(os.environ['BUILD_WORKING_DIRECTORY'])

    visualizer = _ModelVisualizer(visualize_frames=args.visualize_frames,
                                  show_rgbd_sensor=args.show_rgbd_sensor,
                                  triad_length=args.triad_length,
                                  triad_radius=args.triad_radius,
                                  triad_opacity=args.triad_opacity,
                                  browser_new=args.browser_new,
                                  pyplot=args.pyplot,
                                  environment_map=args.environment_map)
    package_map = visualizer.package_map()
    package_map.PopulateFromRosPackagePath()
    for item in args.filename:
        if item.startswith("package://"):
            visualizer.AddModels(url=item)
        else:
            visualizer.AddModels(item)
    visualizer.Run(position=args.position, loop_once=args.loop_once)


if __name__ == '__main__':
    _main()
