from __future__ import absolute_import, division, print_function
from os.path import dirname, join, pardir, realpath
from platform import python_version_tuple

# We specifically do this prior to loading any other pydrake modules, in order
# to get assertion configuration done as early as possible.
from . import common

from . import rbtree

from .pydrake_path import getDrakePath

# Adding searchable path as inferred by pydrake. This assumes that the python
# module has not been moved outside of the installation directory (in which
# the data has also been installed).
path = dirname(__file__)
version = ".".join(python_version_tuple()[:2])
