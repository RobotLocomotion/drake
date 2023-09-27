"""
MeshCat LCM Display Server (MeLDiS)

A standalone program that can display Drake visualizations in MeshCat
by listening for LCM messages that are broadcast by the simulation.

This can stand in for the legacy ``drake_visualizer`` application of
days past.

From a Drake source build, run this as::

  bazel run //tools:meldis &

From a Drake binary release (including pip releases), run this as::

  python3 -m pydrake.visualization.meldis

For binary releases (except for pip) there is also a shortcut available as::

  /opt/drake/bin/meldis

In many cases, passing ``-w`` (i.e., ``--open-window``) to the program will be
convenient::

  bazel run //tools:meldis -- -w &
"""

import argparse
import webbrowser

from pydrake.common import configure_logging as _configure_logging
from pydrake.visualization._meldis import Meldis as _Meldis


def _available_browsers():
    """Returns the list of known webbrowser controller names. There does not
    appear to be a public API for this, so we best-effort call a private one.
    """
    try:
        webbrowser.register_standard_browsers()
        return sorted(webbrowser._browsers)
    except Exception:
        return []


def _main():
    _configure_logging()
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--host", action="store",
        help="The http listen host for MeshCat. If none is given, 'localhost'"
        " will be used by default. In any case, the result will be printed to"
        " the console.")
    parser.add_argument(
        "-p", "--port", action="store", metavar="NUM", type=int,
        help="The http listen port for MeshCat. If none is given, a default"
        " will be chosen and printed to the console.")
    parser.add_argument(
        "-t", "--open-tab", dest="browser_new",
        action="store_const", const=2, default=None,
        help="Open the MeshCat display in a browser tab.")
    parser.add_argument(
        "-w", "--open-window", dest="browser_new",
        action="store_const", const=1, default=None,
        help="Open the MeshCat display in a new browser window.")
    parser.add_argument(
        "--browser", metavar="NAME", choices=_available_browsers(),
        help="Open the MeshCat display using the given browser. "
        "By default, opens as a new window (use --open-tab to override). "
        "When no --browser is provided, the --open-tab or --open-window flags "
        "use the $BROWSER environment variable by default. "
        f"(Available names: %(choices)s)")
    parser.add_argument(
        "--idle-timeout", metavar="TIME", type=float, default=15*60,
        help="When no web browser has been connected for this many seconds,"
        " this program will automatically exit. Set to 0 to run indefinitely.")
    parser.add_argument(
        "--environment_map", metavar="PATH",
        help="Filesystem path to an image to be used as an environment map. "
             "It must be an image type normally used by your browser (e.g., "
             ".jpg, .png, etc.). HDR images are not supported yet."
    )
    args = parser.parse_args()
    meldis = _Meldis(meshcat_host=args.host, meshcat_port=args.port,
                     environment_map=args.environment_map)
    if args.browser is not None and args.browser_new is None:
        args.browser_new = 1
    if args.browser_new is not None:
        url = meldis.meshcat.web_url()
        controller = webbrowser.get(args.browser)
        controller.open(url=url, new=args.browser_new)
    idle_timeout = args.idle_timeout
    if idle_timeout == 0.0:
        idle_timeout = None
    elif idle_timeout < 0.0:
        parser.error("The --idle_timeout cannot be negative.")
    try:
        meldis.serve_forever(idle_timeout=idle_timeout)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    _main()
