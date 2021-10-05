"""Provides deprecation warnings and utilities for triggering warnings.

By default, this sets all `DrakeDeprecationWarnings` to be shown `"once"`,
which overrides any `-W` command-line arguments. To change this behavior, you
can do something like:

>>> import warnings
>>> from pydrake.common.deprecation import DrakeDeprecationWarning
>>> warnings.simplefilter("always", DrakeDeprecationWarning)

If you would like to disable all Drake-related warnings, you may use the
`"ignore"` action for `warnings.simplefilter`.
"""

import os
import re
import sys
import traceback
from types import ModuleType
import warnings

# TODO(eric.cousineau): Make autocomplete ignore `ModuleShim` attributes
# (e.g. `install`).
# TODO(eric.cousineau): Remove ModuleShim once Drake requires Python >= 3.7
# (for PEP 562).


class ModuleShim:
    """Provides a shim for automatically resolving extra variables.

    This can be used to deprecate import alias in modules to simplify
    dependencies.

    See Also:
        https://stackoverflow.com/a/7668273/7829525

    Note:
        This is not necessary in Python >= 3.7 due to PEP 562.
    Warning:
        This will not work if called on a cc module during import (e.g. using
        ExecuteExtraPythonCode). Instead, you should rename the module from
        `{name}` to `_{name}`, and import the symbols into the new module using
        _import_cc_module_vars.
    """

    def __init__(self, orig_module, handler):
        assert hasattr(orig_module, "__all__"), (
            "Please define `__all__` for this module.")
        assert isinstance(orig_module, ModuleType), (
            "{} must be a module".format(orig_module))
        # https://stackoverflow.com/a/16237698/7829525
        object.__setattr__(self, '_orig_module', orig_module)
        object.__setattr__(self, '_handler', handler)
        object.__setattr__(self, '__doc__', orig_module.__doc__)

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
                if str(e):
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

    def __dir__(self):
        # Implemented to provide a useful subset of completions to
        # `rlcompleter`.
        return self._orig_module.__all__

    @classmethod
    def _install(cls, name, handler, *, auto_all=False):
        """Hook into module's attribute accessors and mutators.

        Args:
            name: Module name. Generally should be __name__.
            handler: Function of the form `handler(var)`, where `var` is the
                variable name.
            auto_all: If True, this will override `__all__` with a listing of
                all non-private variables in the module.

        Note:
            This is private such that `install` does not pollute completion
            candidations provided by `rlcompleter` when it iterates through
            `__bases__`.
        """
        old_module = sys.modules[name]
        if auto_all:
            old_module.__all__ = [
                name
                for name in old_module.__dict__
                if not name.startswith("_")
            ]
        new_module = cls(old_module, handler)
        sys.modules[name] = new_module


class DrakeDeprecationWarning(DeprecationWarning):
    """Extends `DeprecationWarning` to permit Drake-specific warnings to
    be filtered by default, without having side effects on other libraries."""
    pass


def _format_deprecation_message(message, *, date=None):
    assert isinstance(message, str), repr(message)
    if date is not None:
        assert isinstance(date, str), repr(date)
        assert date != "None"
        assert _date_pattern.fullmatch(date) is not None, (
            f"Date does not match YYYY-MM-DD pattern: {repr(date)}"
        )
        # N.B. This follows the formatting in `mkdoc.py`.
        message = (
            f"{message} The deprecated code will be removed from Drake on or "
            f"after {date}."
        )
    assert _date_pattern.search(message) is not None, (
        f"Deprecation messages should have a removal date in the form of "
        f"YYYY-MM-DD. Consider passing in the kwarg `date='YYYY-MM-DD' to "
        f"have a preformatted message.\n"
        f"Original message:\n"
        f"{message}"
    )
    return message


def _warn_deprecated(message, *, date=None, stacklevel=2):
    # Logs a deprecation warning message.  Also used by `deprecation_pybind.h`
    # in addition to this file.
    warnings.warn(
        _format_deprecation_message(message, date=date),
        category=DrakeDeprecationWarning,
        stacklevel=stacklevel,
    )


class _DeprecatedDescriptor:
    """Wraps a descriptor to warn that it is deprecated any time it is
    accessed.
    """

    def __init__(self, original, message, *, date=None):
        assert hasattr(original, '__get__'), (
            f"`original` must be a descriptor: {original}"
        )
        self._original = original
        self.__doc__ = self._original.__doc__
        self._message = message
        self._date = date

    def _warn(self):
        _warn_deprecated(self._message, date=self._date, stacklevel=4)

    def __get__(self, obj, objtype):
        self._warn()
        return self._original.__get__(obj, objtype)

    def __set__(self, obj, value):
        self._warn()
        self._original.__set__(obj, value)

    def __delete__(self, obj):
        self._warn()
        self._original.__delete__(obj)


def deprecated(message, *, date=None):
    """Decorator that deprecates a member of a class based on access.

    Args:
        message: Warning message when member is accessed.
        date: (Optional) String of the form "YYYY-MM-DD".
            If supplied, will reformat the message to add the date as is done
            with DRAKE_DEPRECATED and its processing in mkdoc.py. This must be
            present if ``message`` does not contain the date itself.

    Note:
        This differs from other implementations in that it warns on access,
        not when the method is called. For other methods, see the examples in
        https://stackoverflow.com/a/40301488/7829525.

    Use `ModuleShim` for deprecating variables in a module.
    """
    # TODO(eric.cousineau): If possible, distinguish between descriptors and
    # free functions. See PR #15877 for attempt.

    def wrapped(original):
        return _DeprecatedDescriptor(original, message, date=date)

    return wrapped


def install_numpy_warning_filters(force=False):
    """Install warnings filters specific to NumPy."""
    global _installed_numpy_warning_filters
    if _installed_numpy_warning_filters and not force:
        return
    _installed_numpy_warning_filters = True
    # Warnings specific to comparison with `dtype=object` should be raised to
    # errors (#8315, #8491). Without them, NumPy will swallow the errors and
    # make a DeprecationWarning, while returning effectively garbage values
    # (e.g. comparison based on object ID): either a scalar bool or an array of
    # bools (based on what objects are present and the NumPy version).
    # N.B. Using a `module=` regex filter does not work, as the warning is
    # raised from C code, and thus inherits the calling module, which may not
    # be "numpy\..*" (numpy/numpy#10861).
    warnings.filterwarnings(
        "error", category=DeprecationWarning, message="numpy equal will not")
    warnings.filterwarnings(
        "error", category=DeprecationWarning,
        message="elementwise == comparison failed")
    warnings.filterwarnings(
        "error", category=DeprecationWarning,
        message="elementwise != comparison failed")
    # Error changed in 1.16.0
    warnings.filterwarnings(
        "error", category=DeprecationWarning,
        message="elementwise comparison failed")


def deprecated_callable(message, *, date=None):
    """
    Deprecates a callable (a free function or a type/class object) by
    wrapping its invocation to emit a deprecation.

    When possible, use ModuleShim to ensure that a deprecation warning is
    emitted at *import time*, as it can easily be used with pure Python
    modules.

    However, if you are dealing with a C++ module (and are writing code inside
    of `_{module}_extra.py`), you should use this approach.

    Example as decorator:

        @deprecated_callable("Please use `func_y` instead", date="2038-01-19")
        def func_x():
            ...

    Example for alias:

        my_alias = deprecated_callable(
            "Please use `my_original` instead", date="2038-01-19"
        )(my_original)
    """

    def decorator(original):

        def wrapped(*args, **kwargs):
            _warn_deprecated(message, date=date, stacklevel=3)
            return original(*args, **kwargs)

        wrapped.__name__ = original.__name__
        wrapped.__qualname__ = original.__name__
        warning = _format_deprecation_message(message, date=date)
        wrapped.__doc__ = f"Warning:\n\n    {warning}"

        return wrapped

    return decorator


def _forward_callables_as_deprecated(var_dict, m_new, date):
    # Forwards public symbols from `m_new` to `var_dict`, while wrapping
    # each symbol to emit a deprecation warning when it is called.
    # Warning: This assumes all relevant symbols are callable!
    all_public = [x for x in m_new.__dict__ if not x.startswith("_")]
    symbols = getattr(m_new, "__all__", all_public)
    for symbol in symbols:
        new = getattr(m_new, symbol)
        assert hasattr(new, "__call__")
        old_name = var_dict["__name__"]
        message = (
            f"Please use ``{m_new.__name__}.{symbol}`` instead of "
            f"``{old_name}.{symbol}``."
        )
        old = deprecated_callable(message, date=date)(new)
        old.__module__ = old_name
        var_dict[symbol] = old


if os.environ.get("_DRAKE_DEPRECATION_IS_ERROR") == "1":
    # This is used for testing Jupyter notebooks in `jupyter_bazel`.
    warnings.simplefilter('error', DrakeDeprecationWarning)
else:
    warnings.simplefilter('once', DrakeDeprecationWarning)
_installed_numpy_warning_filters = False

# Used to enforce presence of YYYY-MM-DD timestamp for deprecation.
_date_pattern = re.compile(r"\d\d\d\d\-\d\d\-\d\d")
