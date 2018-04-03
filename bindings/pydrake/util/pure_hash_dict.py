"""
Provides accomodations for types which should rely only on hash, rather than
equality comparison.
"""


class _PureHashProxy(object):
    # TODO(eric.cousineau): Copy input object?
    def __init__(self, value):
        self._value = value

    def _get_value(self):
        return self._value

    def __hash__(self):
        return hash(self._value)

    def __eq__(self, other):
        return hash(self._value) == hash(other)

    def __nonzero__(self):
        return bool(self._value)

    value = property(_get_value)


class _DictKeyWrap(dict):
    # Wraps a dictionary's key access.
    def __init__(self, tmp, key_wrap, key_unwrap):
        dict.__init__(self)
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

    def items(self):
        return zip(self.keys(), self.values())

    def keys(self):
        return map(self._key_unwrap, dict.keys(self))

    def iterkeys(self):
        # Non-performant, but sufficient for now.
        return self.keys()

    def iteritems(self):
        # Non-performant, but sufficient for now.
        return self.items()


class PureHashDict(_DictKeyWrap):
    """Implements a dictionary where entries are keyed only by hash value.

    By default, `dict` will use `==` to account for hash collisions. This is
    useful when key comparison via `==` yields a value which is not convertible
    to bool via `__nonzero__`.

    WARNING: This does not constrain key types, so hash collisions are a
    greater possibility if types are mixed.
    """
    def __init__(self, *args, **kwargs):
        tmp = dict(*args, **kwargs)
        _DictKeyWrap.__init__(self, tmp, _PureHashProxy, _PureHashProxy.value)
