#!/usr/bin/env python

import os
import subprocess
import sys


def resolve_path(relpath):
    return os.path.join(os.environ["BAZEL_RUNFILES"], relpath)


def prepend_path(env, relpath):
    os.environ[env] = resolve_path(relpath) + ":" + os.environ.get(env, '')


# TODO(eric.cousineau): Remove these shims if we can teach Bazel how to handle
# these on its own.
# Ensure that LD_LIBRARY_PATH handles `@vtk` on Linux.
if sys.platform.startswith("linux"):
    prepend_path('LD_LIBRARY_PATH', "external/vtk/lib")
# Ensure that PYTHONPATH handles `@vtk` as well.
prepend_path('PYTHONPATH', "external/vtk/lib/python2.7/site-packages")

# Execute binary.
bin_path = resolve_path("external/drake_visualizer/bin/drake-visualizer")
subprocess.check_call([bin_path] + sys.argv[1:])
