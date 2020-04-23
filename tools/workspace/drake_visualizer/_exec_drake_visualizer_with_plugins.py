"""Runs Drake Visualizer with builtin scripts."""
import argparse
import os
import sys

from drake.tools.workspace.drake_visualizer.plugin import (
    __file__ as plugin_init_file,
    AVAILABLE_SCRIPTS,
)


def main(drake_visualizer_real, arg0):
    """Executes `drake_visualizer_real`, intercepting arguments, and passing
    `arg0` to help simplify the usage line from --help."""
    assert os.path.isfile(drake_visualizer_real), (
        f"Must exist: {drake_visualizer_real}")

    available_script_options = ["all"] + AVAILABLE_SCRIPTS
    parser = argparse.ArgumentParser(
        prog="Drake Modifications",
        description=__doc__,
        add_help=False,
    )
    parser.add_argument(
        "--use_builtin_scripts", type=str, default="all", metavar="SCRIPTS",
        help=f"Comma-separated list of Drake's builtin scripts in "
             f"conjunction with Drake Visualizer. "
             f"Default: all. Options: {', '.join(available_script_options)}")
    args, argv = parser.parse_known_args(sys.argv[1:])
    # drake-visualizer greedily consumes arguments, so we must catch them
    # first and pass them as an environment variable.
    os.environ["_DRAKE_VISUALIZER_BUILTIN_SCRIPTS"] = args.use_builtin_scripts
    use_builtin_scripts_file = os.path.join(
        os.path.dirname(plugin_init_file), "use_builtin_scripts.py")

    if "-h" in argv or "--help" in argv:
        parser.print_help()
        print()

    exec_args = [arg0] + argv + ["--script", use_builtin_scripts_file]
    # Due to our usage of `os.exec*`, we need to flush stdout for things like
    # testing with `subprocess.run`. Otherwise, this wrapper's output is lost,
    # and only the real binary's output is captured.
    sys.stdout.flush()
    os.execv(drake_visualizer_real, exec_args)
    assert False, "Should not reach here"
