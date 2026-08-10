# This file contains data types used by the macOS-specific build logic. See
# //tools/wheel:builder for the user interface.

from .common import PythonBinder


class PythonTarget:
    """
    A representation of a Python target, constructed from the binder and Python
    version number tuple.

    Example:
        PythonTarget(PythonBinder.NANOBIND, 3, 2, 1)

    Members:
        python_binder: Which binder to use.
        version_tuple: Target version as a tuple, e.g. (3, 2, 1)
        version_full: Target full version as a string, e.g. '3.2.1'
        version: Target major/minor version as a string, e.g. '3.2'
        tag: Target major/minor version without separators, e.g. '32'

    """

    def __init__(self, python_binder: PythonBinder, *version_parts):
        self.python_binder = python_binder
        pv_parts = tuple(map(str, version_parts))
        self.version_tuple = tuple(version_parts)
        self.version_full = ".".join(pv_parts)
        self.version = ".".join(pv_parts[:2])
        self.tag = "".join(pv_parts[:2])
