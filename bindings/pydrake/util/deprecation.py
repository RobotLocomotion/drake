"""
Provides deprecation warnings and utilities for triggering warnings.

By default, this sets all `DrakeDeprecationWarnings` to be shown `"once"`,
which overrides any `-W` command-line arguments. To change this behavior, you
can do something like:

>>> import warnings
>>> from pydrake.util.deprecation import DrakeDeprecationWarning
>>> warnings.simplefilter("always", DrakeDeprecationWarning)

If you would like to disable all Drake-related warnings, you may use the
`"ignore"` action for `warnings.simplefilter`.
"""

import sys
import traceback
import warnings

# TODO(eric.cousineau): Make autocomplete ignore `ModuleShim` attributes
# (e.g. `install`).


class ModuleShim(object):
    """Provides a shim for automatically resolving extra variables.

    This can be used to deprecate import alias in modules to simplify
    dependencies.

    @see https://stackoverflow.com/a/7668273/7829525
    """

    def __init__(self, orig_module, handler):
        assert hasattr(orig_module, "__all__"), (
            "Please define `__all__` for this module.")
        # https://stackoverflow.com/a/16237698/7829525
        object.__setattr__(self, '_orig_module', orig_module)
        object.__setattr__(self, '_handler', handler)

    def __getattr__(self, name):
        # Use the original module if possible.
        m = self._orig_module
        if hasattr(m, name):
            return getattr(m, name)
        else:
            # Otherwise, use the handler, and store the result.
            try:
                value = self._handler(name)
            except AttributeError as e:
                if e.message:
                    raise e
                else:
                    raise AttributeError(
                        "'module' object has no attribute '{}'".format(name))
            setattr(m, name, value)
            return value

    def __setattr__(self, name, value):
        # Redirect writes to the original module.
        setattr(self._orig_module, name, value)

    def __delattr__(self, name):
        # Redirect deletions to the original module.
        delattr(self._orig_module, name)

    def __repr__(self):
        return repr(self._orig_module)

    @classmethod
    def install(cls, name, handler):
        """ Hook into module's attribute accessors and mutators.
        @param name
            Module name. Generally should be __name__.
        @param handler
            Function of the form `handler(var)`, where `var` is
            the variable name.
        """
        old_module = sys.modules[name]
        new_module = cls(old_module, handler)
        sys.modules[name] = new_module


class DrakeDeprecationWarning(DeprecationWarning):
    """Extends `DeprecationWarning` to permit Drake-specific warnings to
    be filtered by default, without having side effects on other libraries."""
    addendum = ("\n    Please see `help(pydrake.util.deprecation)` " +
                "for more information.")

    def __init__(self, message, *args):
        extra_message = message + DrakeDeprecationWarning.addendum
        DeprecationWarning.__init__(self, extra_message, *args)


class _DeprecatedDescriptor(object):
    """Wraps a descriptor to warn that it is deprecated any time it is
    acccessed."""

    def __init__(self, original, message):
        assert hasattr(original, '__get__'), "`original` must be a descriptor"
        self._original = original
        self.__doc__ = self._original.__doc__
        self._message = message

    def _warn(self):
        warnings.warn(
            self._message, category=DrakeDeprecationWarning, stacklevel=3)

    def __get__(self, obj, objtype):
        self._warn()
        return self._original.__get__(obj, objtype)

    def __set__(self, obj, value):
        self._warn()
        self._original.__set__(obj, value)

    def __delete__(self, obj):
        self._warn()
        self._original.__delete__(obj)


def deprecated(message):
    """Decorator that deprecates a member of a class based on access.

    @param message Warning message when member is accessed.

    @note This differs from other implementations in that it warns on
    access, not when the method is called. For other methods, see
    the examples in https://stackoverflow.com/a/40301488/7829525.

    Use `ModuleShim` for deprecating variables in a module."""
    def wrapped(original):
        return _DeprecatedDescriptor(original, message)

    return wrapped


warnings.simplefilter('once', DrakeDeprecationWarning)
