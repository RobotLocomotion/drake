"""
Provides a semi-transparent layer of C++ const-honoring Python proxies.

This is "semi-transparent" in that all original code should work as is,
*except* for any code that relies on calling `type(obj)`.
If the code can be modified, then it should use `type_extract(obj)` from this
module to be robust against this situation.
"""

import inspect
from types import MethodType

from pydrake.third_party.wrapt import ObjectProxy

# TODO(eric.cousineau): Add mechanism for enabling / disabling const-proxying.

# TODO(eric.cousineau): Determine if propagation for `const` attributes and
# methods is ever useful in pure Python, for things such as `__iter__`,
# `__getitem__`, `__getslice__`, etc.


class ConstError(TypeError):
    """Indicates `const` access violations."""
    pass


class _ConstClassMeta(object):
    # Provides metadata for a given class.
    def __init__(self, cls, owned_properties=None, mutable_methods=None):
        self._cls = cls
        self._owned_properties = set(owned_properties or set())
        self.mutable_methods = set(mutable_methods or set())
        # Add any decorated mutable methods.
        predicate = inspect.isfunction
        methods = inspect.getmembers(cls, predicate=predicate)
        for name, method in methods:
            # TODO(eric.cousineau): Warn if there is a mix of mutable and
            # immutable methods with the same name.
            if getattr(method, '_is_mutable_method', False):
                self.mutable_methods.add(name)
        # Handle inheritance.
        for base_cls in self._cls.__bases__:
            base_meta = _const_metas.get(base_cls)  # Minor cyclic dependency.
            self._owned_properties.update(base_meta._owned_properties)
            self.mutable_methods.update(base_meta.mutable_methods)

    def is_owned_property(self, name):
        # Determines if a property is owned, by name. This implies that the
        # returned value should be `const`.
        return name in self._owned_properties

    def is_mutable_method(self, name):
        # Determines if a method is mutable, by name. This implies that the
        # method should not be called by a const-proxied object.
        # N.B. This would not handle overloads (e.g. differing between
        # `T& get()` and `const T& get() const`).
        # However, if using `pybind11`, the method signatures should play well
        # with custom `type_caster`s that will permit the overloads to
        # organically prevent `const` violations.
        return name in self.mutable_methods


class _ConstClassMetaMap(object):
    # Provides mapping from class to metadata.
    def __init__(self):
        self._meta_map = {}

    def emplace(self, cls, owned_properties=None, mutable_methods=None):
        # Constructs metadata and registers it.
        meta = _ConstClassMeta(cls, owned_properties, mutable_methods)
        return self._add(cls, meta)

    def _add(self, cls, meta):
        assert cls not in self._meta_map
        self._meta_map[cls] = meta
        return meta

    def get(self, cls):
        # Retrieves metadata for a class.
        # Assumes class has not changed if it's already registered.
        meta = self._meta_map.get(cls, None)
        if meta:
            return meta
        else:
            # Construct default.
            return self._add(cls, _ConstClassMeta(cls))


_const_metas = _ConstClassMetaMap()
# Register common mutators.
# N.B. All methods registered will respect inheritance.
# N.B. For `object`, see `_install_object_mutable_methods`.
# @see https://docs.python.org/2.7/reference/datamodel.html#special-method-names  # noqa
_const_metas.emplace(object, mutable_methods={
    # https://docs.python.org/2.7/reference/datamodel.html#customizing-attribute-access  # noqa
    "__setattr__",
    "__delattr__",
    # https://docs.python.org/2.7/reference/datamodel.html#emulating-container-types  # noqa
    "__setitem__",
    "__delitem__",
    "__setslice__",
    "__delslice__",
    # https://docs.python.org/2.7/reference/datamodel.html#emulating-numeric-type  # noqa
    "__iadd__",
    "__isub__",
    "__imul__",
    "__idiv__",
    "__itruediv__",
    "__ifloordiv__",
    "__imod__",
    "__ipow__",
    "__ilshift__",
    "__irshift__",
    "__iand__",
    "__ixor__",
    "__ior__",
    # https://docs.python.org/2.7/reference/datamodel.html#with-statement-context-managers  # noqa
    "__enter__",
    "__exit__",
    })
_const_metas.emplace(list, mutable_methods={
    "append",
    "clear",
    "extend",
    "insert",
    "pop",
    "remove",
    "reverse",
    "sort",
    })
_const_metas.emplace(dict, mutable_methods={
    "clear",
    "pop",
    "popitem",
    "setdefault",
    "update",
    })


class _Const(ObjectProxy):
    # Wraps an object, restricting access to non-mutable methods and
    # properties.
    def __init__(self, wrapped):
        ObjectProxy.__init__(self, wrapped)

    def __getattr__(self, name):
        # Intercepts access to mutable methods, general methods, and owned
        # properties.
        wrapped = object.__getattribute__(self, '__wrapped__')
        meta = _const_metas.get(type(wrapped))
        value = getattr(wrapped, name)
        if meta.is_mutable_method(name):
            # If decorated as a mutable method, raise an error. Do not allow
            # access to the bound method, because the only way this method
            # *should* become usable is to rebind it.
            _raise_mutable_method_error(self, name)
        elif _is_method_of(value, wrapped):
            # Rebind method to const `self` recursively catch basic violations.
            return _rebind_method(value, self)
        elif meta.is_owned_property(name):
            # References (pointer-like things) should not be const, but
            # internal values should.
            return to_const(value)
        else:
            return value

    def __dict_custom__(self):
        # Prevent direct mutation of dictionary.
        return to_const(self.__wrapped__.__dict__)

    def __repr__(self):
        # Insert `const` if using a generic representation.
        out = self.__wrapped__.__repr__()
        if (len(out) >= 2 and out[0] == '<' and out[-1] == '>'):
            return '<const ' + out[1:]
        else:
            return out

    def __str__(self):
        T = type(self.__wrapped__)
        if T.__str__ == object.__str__:
            return self.__repr__()
        else:
            return self.__wrapped__.__str__()


def _install_object_mutable_methods():
    # Installs methods to `_Const` that will always raise an error.
    # N.B. The methods for `object` actually have to be overridden in `_Const`,
    # since neither `__getattr__` nor `__getattribute__` will capture them.
    for name in _const_metas.get(object).mutable_methods:
        def _capture(name):
            def _no_mutable(self, *args, **kwargs):
                _raise_mutable_method_error(self, name)
            _no_mutable.__name__ = name
            setattr(_Const, name, _no_mutable)
        _capture(name)


_install_object_mutable_methods()

_LITERAL_TYPES = {int, float, str, tuple, type(None)}


def _is_immutable(obj):
    # Detects if a type is a immutable (or literal) type.
    return type(obj) in _LITERAL_TYPES


def _is_method_of(func, obj):
    return getattr(func, '__self__', None) is obj


def _rebind_method(bound, new_self):
    return MethodType(bound.__func__, new_self)


def to_const(obj):
    """Converts an object to a const proxy.

    Accepts any object type. If object is immutable or already const, it will
    be passed through.
    """
    if is_const_or_immutable_test(obj):
        return obj
    else:
        return _Const(obj)


def to_mutable(obj, force=False):
    """Converts to a mutable (non-const proxied) object.

    If `force` is False, will throw an error if `obj` is const.
    """
    if is_const_test(obj):
        if not force:
            raise ConstError(
                "Cannot cast const {} to mutable instance"
                .format(type_extract(obj)))
        return obj.__wrapped__
    else:
        return obj


def is_const_test(obj):
    """Determines if `obj` is const-proxied.

    Warning:
        Do NOT use this for branching in production code unless const-proxying
        is guaranteed to be enabled or the branching is designed not to fail
        in this case (the code will always work the same, whether
        const-proxying is enabled or disabled).
    """
    if isinstance(obj, _Const):
        return True
    else:
        return False


def is_const_or_immutable_test(obj):
    """Determines if `obj` is const-proxied or immutable.

    Warning:
        Do NOT use this for branching in production code unless const-proxying
        is guaranteed to be enabled or the branching is designed not to fail
        in this case (the code will always work the same, whether
        const-proxying is enabled or disabled).
    """
    return is_const_test(obj) or _is_immutable(obj)


def type_extract(obj):
    """Extracts type from an object if it's const-proxied; otherwise returns
    direct type.
    """
    if is_const_test(obj):
        return type(obj.__wrapped__)
    else:
        return type(obj)


def _raise_mutable_method_error(obj, name):
    raise ConstError(
        ("'{}' is a mutable method that cannot be called on a " +
         "const {}.").format(name, type_extract(obj)))


def mutable_method(func):
    """Returns a function decorated as mutable.

    This is for decorating methods.
    """
    func._is_mutable_method = True
    return func


def const_decorated(owned_properties=None, mutable_methods=None):
    """Returns a class decorated with const-proxy metadata.

    This is for decorating classes.
    """

    def add_const_meta(cls):
        _const_metas.emplace(cls, owned_properties, mutable_methods)
        return cls

    return add_const_meta
