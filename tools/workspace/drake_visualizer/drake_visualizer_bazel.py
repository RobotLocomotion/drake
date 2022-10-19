"""Runs Drake Visualizer with builtin scripts under Bazel, hacking PYTHONPATH
and LD_LIBRARY_PATH to ensure it runs with the given externals."""

# TODO(eric.cousineau): At some point, environment manipulations should be
# removed.

import os
from os.path import exists, join
import sys

# N.B. Necessary to avoid spewing `__pycache__` when running from `./bazel-bin`
# or `bazel run`.
sys.dont_write_bytecode = True  # noqa

from _drake_visualizer_builtin_scripts import (
    _exec_drake_visualizer_with_plugins,
)

RUNFILES_DIR = os.environ.get("DRAKE_BAZEL_RUNFILES")


def resolve_path(relpath):
    test_paths = [
        # This is for bazel run //tools:drake_visualizer.
        join(RUNFILES_DIR, relpath),
    ]
    if not relpath.startswith("external/"):
        # This is for bazel run @drake//tools:drake_visualizer.
        test_paths.insert(0, join(RUNFILES_DIR, "external/drake", relpath))
    for p in test_paths:
        if exists(p):
            return p
    assert False, (
        "No available paths:\n{}".format("\n".join(test_paths)))


def set_path(key, relpath):
    os.environ[key] = resolve_path(relpath)


def prepend_path(key, relpath):
    os.environ[key] = resolve_path(relpath) + ":" + os.environ.get(key, '')


def main():
    assert RUNFILES_DIR, (
        "This must be called by a script generated using the "
        "`drake_runfiles_binary` macro.")

    # Placate the drake-visualizer logic that puts it into Director mode when
    # DRAKE_RESOURCE_ROOT is set.
    try:
        del os.environ["DRAKE_RESOURCE_ROOT"]
    except KeyError:
        pass

    if sys.platform.startswith("linux"):
        # Ensure that we handle LD_LIBRARY_PATH for @lcm and vtk-8.
        set_path("LD_LIBRARY_PATH", "external/lcm")
        prepend_path("LD_LIBRARY_PATH", "external/drake_visualizer/lib")
    elif sys.platform == "darwin":
        # Ensure that we handle DYLD_LIBRARY_PATH for @lcm.
        set_path("DYLD_LIBRARY_PATH", "external/lcm")

    # Execute wrapper.
    drake_visualizer_real = resolve_path(
        "external/drake_visualizer/bin/drake-visualizer")
    _exec_drake_visualizer_with_plugins(
        drake_visualizer_real=drake_visualizer_real,
        # Ensure the wrapped binary shows 'drake-visualizer' in its usage.
        arg0=drake_visualizer_real,
    )


assert __name__ == "__main__"
main()
