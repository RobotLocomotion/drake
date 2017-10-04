"""Helper for locating data resources when running as a Bazel-compiled python
program.  In particular, when running a program created by a `py_binary()` rule
within a `BUILD.bazel` file, this module provides access to files that were
declared as resources via the `data = [...]` attribute on the `py_binary` rule.
"""

import errno
import os
import sys


def find_data(relpath):
    """Given a relpath like drake/pkg/res.txt or external/repo/pkg/res.txt,
    find the data file and return its path"""
    # Because we are in a py_binary, Bazel's wrapper script sets up our
    # $PYTHONPATH to have our resources somewhere on a sys.path entry.
    for one_path in sys.path:
        possible = os.path.join(one_path, relpath)
        if os.path.exists(possible):
            return possible
    raise IOError(
        errno.ENOENT,
        "Could not find data {}".format(relpath),
        relpath)
