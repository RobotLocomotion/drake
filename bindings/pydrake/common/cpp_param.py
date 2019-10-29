import ctypes
import numpy as np

"""Defines a mapping between Python and alias types, and provides canonical
Python types as they relate to C++.
"""


def _get_type_name(t, verbose):
    # Gets type name as a string.
    # Defaults to just returning the name to shorten template names.
    if verbose and t.__module__ != "__builtin__":
        return t.__module__ + "." + t.__name__
    else:
        return t.__name__


class _StrictMap(object):
    # Provides a map which may only add a key once.
    def __init__(self):
        self._values = dict()

    def add(self, key, value):
        assert key not in self._values, "Already added: {}".format(key)
        self._values[key] = value

    def get(self, key, default):
        return self._values.get(key, default)


class _ParamAliases(object):
    # Registers aliases for a set of objects. This will be used for template
    # parameters.
    def __init__(self):
        self._to_canonical = _StrictMap()
        self._register_common()

    def _register_common(self):
        # Register common Python aliases relevant for C++.
        self.register(float, [np.double, ctypes.c_double])
        self.register(np.float32, [ctypes.c_float])
        self.register(int, [np.int32, ctypes.c_int32])
        self.register(np.uint8, [ctypes.c_uint8])
        self.register(np.int16, [ctypes.c_int16])
        self.register(np.uint32, [ctypes.c_uint32])
        self.register(np.int64, [ctypes.c_int64])

    def register(self, canonical, aliases):
        # Registers a set of aliases to a canonical value.
        for alias in aliases:
            self._to_canonical.add(alias, canonical)

    def is_aliased(self, alias):
        # Determines if a parameter is aliased / registered.
        return self._to_canonical.get(alias, None) is not None

    def get_canonical(self, alias):
        # Gets registered canonical parameter if it is aliased; otherwise
        # return the same parameter.
        return self._to_canonical.get(alias, alias)

    def get_name(self, alias):
        # Gets string for an alias.
        canonical = self.get_canonical(alias)
        if isinstance(canonical, type):
            return _get_type_name(canonical, False)
        else:
            # For literals.
            return str(canonical)


# Create singleton instance.
_param_aliases = _ParamAliases()


def get_param_canonical(param):
    """Gets the canonical types for a set of Python types (canonical as in
    how they relate to C++ types.
    """
    return tuple(map(_param_aliases.get_canonical, param))


def get_param_names(param):
    """Gets the canonical type names for a set of Python types (canonical as
    in how they relate to C++ types.
    """
    return tuple(map(_param_aliases.get_name, param))
