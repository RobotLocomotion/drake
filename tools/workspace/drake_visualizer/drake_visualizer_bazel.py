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

    # Stub out pydrake (refer to our ./BUILD.bazel comments for rationale).
    # If pydrake is already on the path, for instance because intentionally
    # included as a dependency, then the stub will not be added to avoid
    # conflicting.
    #
    # We add it to PYTHONPATH within the script, rather than
    # `imports = ["stub"]` on the stub py_library, to avoid any other target
    # accidentally pulling in the stubbed pydrake onto its PYTHONPATH.  Only
    # the visualizer, when launched via this wrapper script, should employ the
    # stub.
    try:
        import pydrake
    except ImportError:
        prepend_path("PYTHONPATH", "tools/workspace/drake_visualizer/stub")

    # Don't use DRAKE_RESOURCE_ROOT; the stub getDrakePath should always win.
    # This also placates the drake-visualizer logic that puts it into Director
    # mode when DRAKE_RESOURCE_ROOT is set (thus requiring more than just
    # getDrakePath).
    try:
        del os.environ["DRAKE_RESOURCE_ROOT"]
    except KeyError:
        pass

    if sys.platform.startswith("linux"):
        # Ensure that we handle LD_LIBRARY_PATH for @lcm and @vtk and
        # PYTHONPATH for @vtk.
        set_path("LD_LIBRARY_PATH", "external/lcm")
        prepend_path("LD_LIBRARY_PATH", "external/vtk/lib")
        prepend_path(
            "PYTHONPATH", "external/vtk/lib/python{}.{}/site-packages".format(
                *sys.version_info[:2]))
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
