import os
import subprocess
import sys


def resolve_path(relpath):
    abspath = os.path.join(runfiles_dir, relpath)
    assert os.path.exists(abspath), "Path does not exist: {}".format(abspath)
    return abspath


def prepend_path(key, relpath):
    os.environ[key] = resolve_path(relpath) + ":" + os.environ.get(key, '')


runfiles_dir = os.environ.get("DRAKE_BAZEL_RUNFILES")
assert runfiles_dir, (
    "This must be called by a script generated using the " +
    "`drake_runfiles_binary` macro.")

# TODO(eric.cousineau): Remove these shims if we can teach Bazel how to handle
# these on its own.
if sys.platform.startswith("linux"):
    # Ensure that we handle LD_LIBRARY_PATH and PYTHONPATH for `@vtk`.
    prepend_path('LD_LIBRARY_PATH', "external/vtk/lib")
    prepend_path('PYTHONPATH', "external/vtk/lib/python2.7/site-packages")

# Execute binary.
bin_path = resolve_path("external/drake_visualizer/bin/drake-visualizer")
args = [bin_path] + sys.argv[1:]
os.execv(bin_path, args)
