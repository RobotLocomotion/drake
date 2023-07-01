"""
MeshCat LCM Display Server (MeLDiS)

A standalone program that can display Drake visualizations in MeshCat
by listening for LCM messages that are broadcast by the simulation.

This can stand in for the legacy ``drake-visualizer`` application of
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
        "--idle-timeout", metavar="TIME", type=float, default=15*60,
        help="When no web browser has been connected for this many seconds,"
        " this program will automatically exit. Set to 0 to run indefinitely.")
    args = parser.parse_args()
    meldis = _Meldis(meshcat_host=args.host, meshcat_port=args.port)
    if args.browser_new is not None:
        url = meldis.meshcat.web_url()
        webbrowser.open(url=url, new=args.browser_new)
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
