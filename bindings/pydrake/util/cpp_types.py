from __future__ import absolute_import, print_function

import ctypes
import numpy as np

"""
@file
Defines a mapping between Python and C++ types, and provides canonical Python
types as they relate to C++.
"""

# Define these first, as they are used in `cpp_types_py.cc`
# (transitively, `cpp_types_pybind.cc`).


def _get_type_name(t):
    # Gets type name as a string.
    prefix = t.__module__ + "."
    if prefix == "__builtin__.":
        prefix = ""
    return prefix + t.__name__


class _StrictMap(object):
    # Provides a map which may only add a key once.
    def __init__(self):
        self._values = dict()

    def add(self, key, value):
        assert key not in self._values, "Already added: {}".format(key)
        self._values[key] = value

    def get(self, key, default):
        return self._values.get(key, default)


class _TypeRegistry(object):
    def __init__(self):
        self._cpp_to_py_canonical = _StrictMap()
        self._py_to_py_canonical = _StrictMap()

    def register_cpp(self, py_canonical, cpp_type):
        self._cpp_to_py_canonical.add(cpp_type, py_canonical)

    def register_py(self, py_types):
        py_canonical = py_types[0]
        for py_type in py_types:
            self._py_to_py_canonical.add(py_type, py_canonical)

    def get_type_canonical(self, py_type):
        # Get registered canonical type if there is a mapping; otherwise return
        # original type.
        return self._py_to_py_canonical.get(py_type, py_type)

    def get_name(self, py_type):
        return _get_type_name(self.get_type_canonical(py_type))

    def get_type_canonical_from_cpp(self, cpp_type):
        # Return None so that C++ can query pybind.
        return self._cpp_to_py_canonical.get(cpp_type, None)


_type_registry = _TypeRegistry()

# Register canonical Python types.
_py_types_set = (
    (float, ctypes.c_double, np.double),
    (np.float32, ctypes.c_float),
    (int, np.int32, ctypes.c_int32),
    (np.uint32, ctypes.c_uint32),
)
map(_type_registry.register_py, _py_types_set)

# Import C++ types.
import pydrake.util._cpp_types_py  # noqa


def get_types_canonical(param):
    """Gets the canonical types for a set of Python types (canonical as in
    how they relate to C++ types. """
    return tuple(map(_type_registry.get_type_canonical, param))


def get_type_names(param):
    """Gets the canonical type names for a set of Python types (canonical as in
    how they relate to C++ types. """
    return tuple(map(_type_registry.get_name, param))
