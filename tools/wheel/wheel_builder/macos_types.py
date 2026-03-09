# This file contains data types used by the macOS-specific build logic. See
# //tools/wheel:builder for the user interface.


class PythonTarget:
    """
    A representation of a Python version target, constructed from the version
    number tuple.

    Example:
        PythonTarget(3, 2, 1)

    Members:
        version_tuple: Target version as a tuple, e.g. (3, 2, 1)
        version_full: Target full version as a string, e.g. '3.2.1'
        version: Target major/minor version as a string, e.g. '3.2'
        tag: Target major/minor version without separators, e.g. '32'
    """

    def __init__(self, *version_parts):
        pv_parts = tuple(map(str, version_parts))
        self.version_tuple = tuple(version_parts)
        self.version_full = ".".join(pv_parts)
        self.version = ".".join(pv_parts[:2])
        self.tag = "".join(pv_parts[:2])
