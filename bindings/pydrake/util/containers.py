"""
Provides extensions for containers of Drake-related objects.
"""


class _EqualityProxyBase(object):
    # TODO(eric.cousineau): Copy input object to preserve key immutability?
    def __init__(self, value):
        self._value = value

    def _get_value(self):
        return self._value

    def __hash__(self):
        return hash(self._value)

    def __eq__(self, other):
        raise NotImplemented

    def __nonzero__(self):
        return bool(self._value)

    value = property(_get_value)


class _DictKeyWrap(dict):
    # Wraps a dictionary's key access.
    def __init__(self, tmp, key_wrap, key_unwrap):
        dict.__init__(self)
        # N.B. Passing properties to these will cause an issue. This can be
        # sidestepped by storing the properties in a `dict`.
        self._key_wrap = key_wrap
        self._key_unwrap = key_unwrap
        for key, value in tmp.iteritems():
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
        return [self._key_unwrap(key) for key in dict.iterkeys(self)]

    def iterkeys(self):
        # Non-performant, but sufficient for now.
        return self.keys()

    def iteritems(self):
        # Non-performant, but sufficient for now.
        return self.items()

    def raw(self):
        """Returns a dict with the original keys.
        N.B. Copying to a `dict` will maintain the proxy keys."""
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

        tmp = dict(*args, **kwargs)
        _DictKeyWrap.__init__(self, tmp, Proxy, Proxy._get_value)
