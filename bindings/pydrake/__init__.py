from __future__ import absolute_import, division, print_function
from os.path import abspath
from platform import python_version_tuple
from sys import stderr

# As a workaround to bazelbuild/bazel#4594, we can use this spelling to load
# a properly-linked version of `libdrake.so`. Once this is loaded, all of the
# Python C-extension libraries that depend on it (and its dependencies) will
# load properly.
try:
    import external.drake.bindings.pydrake
except ImportError:
    pass

# We specifically load `common` prior to loading any other pydrake modules,
# in order to get assertion configuration done as early as possible.
from . import common
from .util.deprecation import ModuleShim


def getDrakePath():
    # Compatibility alias.
    return abspath(common.GetDrakePath())


__all__ = ['common', 'getDrakePath']


def _getattr_handler(name):
    # Deprecate direct usage of "rbtree" without having imported the module.
    if name == "rbtree":
        # N.B. Calling `from . import rbtree` will cause recursion, because
        # `from x import y` uses `hasattr(x, "y")`, which merely checks if
        # `getattr(x, "y")` is exception-free.
        import pydrake.rbtree
        return pydrake.rbtree
    else:
        raise AttributeError()


ModuleShim.install(__name__, _getattr_handler)
