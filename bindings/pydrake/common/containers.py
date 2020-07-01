"""Provides extensions for containers of Drake-related objects."""

import numpy as np


class _EqualityProxyBase:
    # Wraps an object with a non-compliant `__eq__` operator (returns a
    # non-bool convertible expression) with a custom compliant `__eq__`
    # operator.
    def __init__(self, value):
        self._value = value

    def _get_value(self):
        return self._value

    def __hash__(self):
        return hash(self._value)

    def __eq__(self, other):
        raise NotImplemented("Abstract method")

    value = property(_get_value)


class _DictKeyWrap(dict):
    # Wraps a dictionary's key access. For a key of a type `TOrig`, this
    # dictionary will provide a key of type `TProxy`, that should proxy the
    # original key.
    def __init__(self, dict_in, key_wrap, key_unwrap):
        # @param dict_in Dictionary with keys of types TOrig (not necessarily
        # homogeneous).
        # @param key_wrap Functor that maps from TOrig -> TProxy.
        # @param key_unwrap Functor that maps from TProxy -> TOrig.
        dict.__init__(self)
        # N.B. Passing properties to these will cause an issue. This can be
        # sidestepped by storing the properties in a `dict`.
        self._key_wrap = key_wrap
        self._key_unwrap = key_unwrap
        for key, value in dict_in.items():
            self[key] = value

    def __setitem__(self, key, value):
        return dict.__setitem__(self, self._key_wrap(key), value)

    def __getitem__(self, key):
        return dict.__getitem__(self, self._key_wrap(key))

    def __delitem__(self, key):
        return dict.__delitem__(self, self._key_wrap(key))

    def __contains__(self, key):
        return dict.__contains__(self, self._key_wrap(key))

    def items(self):
        return zip(self.keys(), self.values())

    def keys(self):
        return (self._key_unwrap(key) for key in dict.keys(self))

    def raw(self):
        """Returns a dict with the original keys.

        Note:
            Copying to a `dict` will maintain the proxy keys.
        """
        return dict(self.items())


class EqualToDict(_DictKeyWrap):
    """Implements a dictionary where keys are compared using type and
    `lhs.EqualTo(rhs)`.
    """
    def __init__(self, *args, **kwargs):

        class Proxy(_EqualityProxyBase):
            def __eq__(self, other):
                T = type(self.value)
                return (isinstance(other.value, T)
                        and self.value.EqualTo(other.value))

            # https://stackoverflow.com/a/1608907/7829525
            __hash__ = _EqualityProxyBase.__hash__

        dict_in = dict(*args, **kwargs)
        _DictKeyWrap.__init__(self, dict_in, Proxy, Proxy._get_value)


class NamedViewBase:
    """Base for classes generated by ``namedview``.

    Inspired by: https://bitbucket.org/ericvsmith/namedlist
    """

    _fields = None  # To be specified by inherited classes.

    def __init__(self, value):
        """Creates a view on ``value``. Any mutations on this instance will be
        reflected in ``value``, and any mutations on ``value`` will be
        reflected in this instance."""
        assert self._fields is not None, (
            "Class must be generated by ``namedview``")
        assert len(self._fields) == len(value)
        object.__setattr__(self, '_value', value)

    @classmethod
    def get_fields(cls):
        """Returns all fields for this class or object."""
        return cls._fields

    def __getitem__(self, i):
        return self._value[i]

    def __setitem__(self, i, value_i):
        self._value[i] = value_i

    def __setattr__(self, name, value):
        """Prevent setting additional attributes."""
        if not hasattr(self, name):
            raise AttributeError("Cannot add attributes!")
        object.__setattr__(self, name, value)

    def __len__(self):
        return len(self._value)

    def __iter__(self):
        return iter(self._value)

    def __array__(self):
        """Proxy for use with NumPy."""
        return np.asarray(self._value)

    def __repr__(self):
        """Provides human-readable breakout of each field and value."""
        value_strs = []
        for i, field in enumerate(self._fields):
            value_strs.append("{}={}".format(field, repr(self[i])))
        return "{}({})".format(self.__class__.__name__, ", ".join(value_strs))

    @staticmethod
    def _item_property(i):
        # Maps an item (at a given index) to a property.
        return property(
            fget=lambda self: self[i],
            fset=lambda self, value: self.__setitem__(i, value))


def namedview(name, fields):
    """
    Creates a class that is a named view with given ``fields``. When the class
    is instantiated, it must be given the object that it will be a proxy for.
    Similar to ``namedtuple``.

    Example::
        MyView = namedview("MyView", ('a', 'b'))

        value = np.array([1, 2])
        view = MyView(value)
        view.a = 10  # `value` is now [10, 2]
        value[1] = 100  # `view` is now [10, 100]
        view[:] = 3  # `value` is now [3, 3]

    For more details, see ``NamedViewBase``.
    """
    base_cls = (NamedViewBase,)
    type_dict = dict(_fields=tuple(fields))
    for i, field in enumerate(fields):
        type_dict[field] = NamedViewBase._item_property(i)
    cls = type(name, base_cls, type_dict)
    return cls
