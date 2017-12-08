from __future__ import absolute_import, division, print_function
from os.path import dirname, join, pardir, realpath
from platform import python_version_tuple
from sys import stderr

# We specifically load `common` prior to loading any other pydrake modules,
# in order to get assertion configuration done as early as possible.
from . import common
from .util import ModuleShim


def _init_path():
    # Adding searchable path as inferred by pydrake. This assumes that the
    # python module has not been moved outside of the installation directory
    # (in which the data has also been installed).
    path = dirname(__file__)
    version = ".".join(python_version_tuple()[:2])
    # In the install tree. pydrake Python module is in
    # `lib/python2.7/site-packages/pydrake/` whereas the data is installed in
    # `share/drake`. From the current file location, the data is 4 directories
    # up. If pydrake is not in the expected directory, `path` is not added to
    # the resource search path.
    if path.endswith("lib/python" + version + "/site-packages/pydrake"):
        common.AddResourceSearchPath(
            realpath(join(path,
                          pardir, pardir, pardir, pardir,
                          "share/drake"))
        )


def getDrakePath():
    # Compatibility alias.
    return common.GetDrakePath()


_init_path()
__all__ = ['common', 'getDrakePath']


def _getattr_handler(name, import_type):
    # Deprecate direct usage of "rbtree" without having imported the module.
    if name == "rbtree":
        if import_type == "direct":
            stderr.write(
                "`import pydrake; pydrake.rbtree` will soon be deprecated." +
                "\n  Please use `import pydrake.rbtree` or `from pydrake " +
                "import rbtree` instead.\n")
        # N.B. Calling `from . import rbtree` will cause recursion, because
        # `from x import y` uses `hasattr(x, "y")`, which merely checks if
        # `getattr(x, "y")` is exception-free.
        import pydrake.rbtree
        return pydrake.rbtree
    else:
        raise AttributeError()


ModuleShim.install(__name__, _getattr_handler)
