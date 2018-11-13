"""Provides extensions for containers of Drake-related objects."""

import six


class _EqualityProxyBase(object):
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

    def __nonzero__(self):
        return bool(self._value)

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
        for key, value in six.iteritems(dict_in):
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
        # `six.iterkeys` will not constrain the call to use `dict` methods.
        if six.PY2:
            keys_iter = dict.iterkeys(self)
        else:
            keys_iter = dict.keys(self)
        return [self._key_unwrap(key) for key in keys_iter]

    def iterkeys(self):
        # Non-performant, but sufficient for now.
        return self.keys()

    def iteritems(self):
        # Non-performant, but sufficient for now.
        return self.items()

    def raw(self):
        """Returns a dict with the original keys.

        Note:
            Copying to a `dict` will maintain the proxy keys.
        """
        return dict(self.iteritems())


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
