"""
Defines a mapping between Python and alias types, and provides canonical
Python types as they relate to C++.
"""

import ctypes

import numpy as np


def _get_type_name(t, verbose):
    # Gets type name as a string.
    # Defaults to just returning the name to shorten template names.
    if verbose and t.__module__ != "__builtin__":
        return t.__module__ + "." + t.__name__
    else:
        return t.__name__


class _StrictMap:
    # Provides a map which may only add a key once.
    def __init__(self):
        self._values = dict()

    def add(self, key, value):
        assert key not in self._values, "Already added: {}".format(key)
        self._values[key] = value

    def get(self, key, default):
        return self._values.get(key, default)


class _ParamAliases:
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


class _Generic:
    """
    Provides a way to denote unique "classes" for C++ generics that do not
    normally convert to unique types in Python (via pybind11).

    As an example, pybind11 casts ``std::vector<T>`` to ``list()``, but we may
    still want to associate a unique type with it for registration in
    templates. We can do this by creating a unique class (or object),
    ``List[T]``.

    The ``typing`` module in Python provides generics like this; however, the
    API does not admit easy inspection, at least in Python 3.6 and 3.8, thus we
    reinvent a smaller wheel.
    """
    def __init__(self, name, factory, num_param):
        self._name = name
        self._factory = factory
        self._num_param = num_param
        # TODO(eric.cousineau): Rather than caching, consider allowing this to
        # `hash` the same as `typing`. As an example, both `typing.List` and
        # this `List` implementation could be used to retrieve an
        # implementation. However, that would also may need to be handled in
        # `get_canonical`.
        self._instantiations = {}

    class _Instantiation:
        # TODO(eric.cousineau): Return a class if it messes up downstream APIs.
        def __init__(self, generic, param):
            param_str = ', '.join(get_param_names(param))
            self._full_name = f"{generic._name}[{param_str}]"
            self._factory = generic._factory

        def __call__(self, *args, **kwargs):
            return self._factory(*args, **kwargs)

        def __repr__(self):
            return self._full_name

    def __getitem__(self, param):
        if not isinstance(param, tuple):
            param = (param,)
        if len(param) != self._num_param:
            raise RuntimeError(
                f"{self} can only accept {self._num_param} parameter(s)")
        param = get_param_canonical(param)
        # Rather than implement hashing, simply cache instantiations.
        instantiation = self._instantiations.get(param)
        if instantiation is None:
            instantiation = self._Instantiation(self, param)
            self._instantiations[param] = instantiation
        return instantiation

    def __repr__(self):
        return f"<Generic {self._name}>"


List = _Generic("List", factory=list, num_param=1)
