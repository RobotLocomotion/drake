"""
Includes utilitiy functions for plugins, as well as functionality for ensuring
that Drake Visualizer can be launched with builtin scripts wrapped in for both
Bazel and the install tree.

N.B. This file should be importable by any Drake Python target, and should
not depend on anything that requires Drake Visualizer / director.
"""

import argparse
from functools import wraps
import os
import subprocess
import sys
from warnings import warn
import weakref

# N.B. Keep this in sync (with the same ordering) as the keys defined in
# `available` in `use_builtin_scripts.py`.
AVAILABLE_SCRIPTS = [
    "experimental_deformable_mesh",
    "frame",
    "hydroelastic_contact",
    "image",
    "point_cloud",
    "point_pair_contact",
    "time",
    "grid_wireframe",
    "limit_clipping_range",
]


def scoped_singleton_func(f):
    """Decorates a function that should only be called once
    (e.g. `init_visualization`). A weak-reference is maintained, such that the
    lifetime of the object is not affected by this decorator.
    """
    state = dict(called=False, args=None, result_ref=None)

    @wraps(f)
    def wrapped(*args, **kwargs):
        result_ref = state["result_ref"]
        is_result_valid = result_ref and result_ref() is not None
        if state["called"] and is_result_valid:
            if (args, kwargs) != state["args"]:
                warn(("Called a singleton function {} with different "
                      "arguments!").format(f))
            return result_ref()
        result = f(*args, **kwargs)
        state.update(
            called=True, args=(args, kwargs),
            result_ref=weakref.ref(result))
        return result

    return wrapped


def _exec_drake_visualizer_with_plugins(drake_visualizer_real, arg0):
    """Executes `drake_visualizer_real`, intercepting arguments, and passing
    `arg0` to help simplify the usage line from --help."""
    assert os.path.isfile(drake_visualizer_real), (
        f"Must exist: {drake_visualizer_real}")

    available_script_options = ["all"] + AVAILABLE_SCRIPTS
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--use_builtin_scripts", type=str, default="all", metavar="SCRIPTS",
        help=f"Comma-separated list of Drake's builtin scripts to use in "
             f"conjunction with Drake Visualizer. "
             f"Default: all. Options: {', '.join(available_script_options)}")
    args, argv = parser.parse_known_args(sys.argv[1:])
    # drake-visualizer greedily consumes arguments, so we must catch them
    # first and pass them as an environment variable.
    os.environ["_DRAKE_VISUALIZER_BUILTIN_SCRIPTS"] = args.use_builtin_scripts
    use_builtin_scripts_file = os.path.join(
        os.path.dirname(__file__), "use_builtin_scripts.py")

    if "-h" in argv or "--help" in argv:
        # Show real help, followed by our wrapper help.
        # TODO(eric.cousineau): Fix indentation if it ever matters.
        original_help = subprocess.run(
            [drake_visualizer_real, "--help"],
            check=True, encoding="utf8",
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT).stdout
        original_help = original_help.replace(
            os.path.basename(drake_visualizer_real), "drake-visualizer")
        wrapper_help = parser.format_help().splitlines()
        detail_index = wrapper_help.index("optional arguments:")
        print(original_help)
        print("\n".join(wrapper_help[detail_index + 1:]))
        sys.exit(0)

    exec_args = [arg0] + argv + ["--script", use_builtin_scripts_file]
    # Due to our usage of `os.exec*`, we need to flush stdout for things like
    # testing with `subprocess.run`. Otherwise, this wrapper's output is lost,
    # and only the real binary's output is captured.
    sys.stdout.flush()
    os.execv(drake_visualizer_real, exec_args)
    assert False, "Should not reach here"
