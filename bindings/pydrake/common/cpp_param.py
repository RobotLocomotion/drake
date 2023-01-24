"""
Defines a mapping between Python and alias types, and provides canonical
Python types as they relate to C++.
"""

import ctypes
import sys
import typing

import numpy as np

from pydrake.common import _MangledName, pretty_class_name


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
        self.register(np.int16, [ctypes.c_int16])
        self.register(np.int64, [ctypes.c_int64])
        self.register(np.uint8, [ctypes.c_uint8])
        self.register(np.uint16, [ctypes.c_uint16])
        self.register(np.uint32, [ctypes.c_uint32])
        self.register(np.uint64, [ctypes.c_uint64])

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

    @staticmethod
    def _resugar_typing_shortcuts(origin_name, arg_names):
        # Re-sugars typing.Union[T, NoneType] into typing.Optional[T] in case
        # that's what the origin and arg names refer to. Otherwise, returns the
        # data unchanged.
        if (origin_name == "typing.Union"
                and len(arg_names) == 2
                and arg_names[-1] == "NoneType"):
            origin_name = "typing.Optional"
            arg_names = arg_names[:1]
        return origin_name, arg_names

    def get_name(self, alias, *, mangle):
        # Gets string for an alias.
        canonical = self.get_canonical(alias)
        if typing.get_origin(alias) is not None:
            origin = typing.get_origin(alias)
            args = typing.get_args(alias)
            origin_name = self.get_name(origin, mangle=mangle)
            arg_names = [self.get_name(x, mangle=mangle) for x in args]
            origin_name, arg_names = self._resugar_typing_shortcuts(
                origin_name, arg_names)
            result = f"{origin_name}[{','.join(arg_names)}]"
            if mangle:
                result = _MangledName.mangle(result)
            return result
        elif isinstance(canonical, type):
            if mangle:
                return canonical.__name__
            else:
                return pretty_class_name(canonical)
        else:
            # For literals.
            result = str(canonical)
            if mangle:
                result = _MangledName.mangle(result)
            return result


# Create singleton instance.
_param_aliases = _ParamAliases()


def get_param_canonical(param):
    """Gets the canonical types for a set of Python types (canonical as in
    how they relate to C++ types).
    """
    return tuple(map(_param_aliases.get_canonical, param))


def get_param_names(param, *, mangle=False):
    """Gets the canonical type names for a set of Python types (canonical as
    in how they relate to C++ types).

    The ``mangle`` controls whether we use the nice name or the mangled name;
    see cpp_template.TemplateBase._instantiation_name for details.
    """
    return tuple(_param_aliases.get_name(x, mangle=mangle) for x in param)


class _Generic:
    """Provides a C++-compatible way to denote generic types. This uses the
    same type classes as Python's built-in generics (e.g., `typing.Union`) but
    is careful to canonicalize any type aliases in params during instantiation.
    """
    def __init__(self, name, instantiator, num_params):
        self._name = name
        self._instantiator = instantiator
        self._num_params = num_params

    def __getitem__(self, params):
        if not isinstance(params, tuple):
            params = (params,)
        if self._num_params is not None and len(params) != self._num_params:
            raise RuntimeError(
                f"{self._name}[] requires exactly "
                f"{self._num_params} type parameter(s)")
        params_canonical = get_param_canonical(params)
        return self._instantiator(params_canonical)

    def __repr__(self):
        return f"<Generic {self._name}>"


def _has_pep585():
    return sys.version_info[:2] >= (3, 9)


def _dict_instantiator(params):
    # Backport PEP-585 support into Python 3.8 and earlier.
    if _has_pep585():
        result = dict[params]
    else:
        result = typing.Dict[params]
        # We need the result to be a callable type to match PEP-585 (i.e.,
        # calling it must return a fresh `dict` object.)
        result._inst = True
    return result


def _list_instantiator(params):
    # Backport PEP-585 support into Python 3.8 and earlier.
    if _has_pep585():
        result = list[params]
    else:
        result = typing.List[params]
        # We need the result to be a callable type to match PEP-585 (i.e.,
        # calling it must return a fresh `list` object.)
        result._inst = True
    return result


def _optional_instantiator(params):
    # Unpack the tuple into the (single) argument required by typing.Optional.
    (param,) = params
    return typing.Optional[param]


# A generic type `dict[KT, VT]` for the C++ class std::map<KT, VT>.
Dict = _Generic("Dict", _dict_instantiator, 2)

# A generic type `list[T]` for the C++ class std::vector<T>.
List = _Generic("List", _list_instantiator, 1)

# A generic type `typing.Optional[T]` for the C++ class std::optional<T>.
# Note that `typing` de-sugars this to be `typing.Union[T, NoneType]`.
Optional = _Generic("Optional", _optional_instantiator, 1)

# A generic type `typing.Union[X, ...]` for the C++ class std::variant<X, ...>.
Union = _Generic("Union", typing.Union.__getitem__, None)
