from __future__ import print_function

import argparse
import os
from os.path import exists, join
import subprocess
import sys


def resolve_path(relpath):
    test_paths = [
        # This is for bazel run //tools:drake_visualizer.
        join(runfiles_dir, relpath),
    ]
    if not relpath.startswith("external/"):
        # This is for bazel run @drake//tools:drake_visualizer.
        test_paths.insert(0, join(runfiles_dir, "external/drake", relpath))
    for p in test_paths:
        if exists(p):
            return p
    assert False, (
        "No available paths:\n{}".format("\n".join(test_paths)))


def set_path(key, relpath):
    os.environ[key] = resolve_path(relpath)


def prepend_path(key, relpath):
    os.environ[key] = resolve_path(relpath) + ":" + os.environ.get(key, '')


def extract_use_builtin_scripts(argv):
    # drake-visualizer greedily consumes arguments, so we must catch them
    # first and pass them as an environment variable.
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--use_builtin_scripts", type=str, default="all")
    args, argv = parser.parse_known_args(argv)
    os.environ["_DRAKE_VISUALIZER_BUILTIN_SCRIPTS"] = args.use_builtin_scripts
    return argv


assert __name__ == "__main__", __name__
runfiles_dir = os.environ.get("DRAKE_BAZEL_RUNFILES")
assert runfiles_dir, (
    "This must be called by a script generated using the " +
    "`drake_runfiles_binary` macro.")

# Stub out pydrake (refer to our ./BUILD.bazel comments for rationale).
#
# We add it to PYTHONPATH within the script, rather than `imports = ["stub"]`
# on the stub py_library, to avoid any other target accidentally pulling in the
# stubbed pydrake onto its PYTHONPATH.  Only the visualizer, when launched via
# this wrapper script, should employ the stub.
prepend_path("PYTHONPATH", "tools/workspace/drake_visualizer/stub")

# Don't use DRAKE_RESOURCE_ROOT; the stub getDrakePath should always win.  This
# also placates the drake-visualizer logic that puts it into Director mode when
# DRAKE_RESOURCE_ROOT is set (thus requiring more than just getDrakePath).
try:
    del os.environ["DRAKE_RESOURCE_ROOT"]
except KeyError:
    pass

# TODO(eric.cousineau): Remove these shims if we can teach Bazel how to handle
# these on its own.
if sys.platform.startswith("linux"):
    # Ensure that we handle LD_LIBRARY_PATH for @lcm and @vtk and PYTHONPATH
    # for @vtk.
    set_path("LD_LIBRARY_PATH", "external/lcm")
    prepend_path("LD_LIBRARY_PATH", "external/vtk/lib")
    # TODO(eric.cousineau): Ensure that Drake Visualizer works even when Bazel
    # uses a separate version of Python.
    prepend_path("PYTHONPATH", "external/vtk/lib/python2.7/site-packages")
elif sys.platform == "darwin":
    # Ensure that we handle DYLD_LIBRARY_PATH for @lcm.
    set_path("DYLD_LIBRARY_PATH", "external/lcm")

# Build arguments, handling builtin scripts, and execute.
use_builtin_scripts = resolve_path(
    "tools/workspace/drake_visualizer/plugin/use_builtin_scripts.py")
bin_path = resolve_path("external/drake_visualizer/bin/drake-visualizer")
args = [bin_path] + extract_use_builtin_scripts(sys.argv[1:])
args += ["--script", use_builtin_scripts]
os.execv(bin_path, args)
