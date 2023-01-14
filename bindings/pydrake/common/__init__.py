import collections
import functools
import inspect
import logging as _logging

import numpy as np

from ._module_py import *
from ._module_py import _set_log_level

_root_logger = _logging.getLogger()
_drake_logger = _logging.getLogger("drake")


def _python_level_to_spdlog(level: int):
    """Returns the spdlog string level that enables C++ logging at the given
    Python logging level, for use with pydrake.common._set_log_level.
    """
    if level >= _logging.CRITICAL:
        return "critical"
    if level >= _logging.ERROR:
        return "err"
    if level >= _logging.WARNING:
        return "warn"
    if level >= _logging.INFO:
        return "info"
    if level >= _logging.DEBUG:
        return "debug"
    return "trace"


def _sync_spdlog_level():
    """Updates Drake's C++ spdlog level threshold to match the effective level
    of Drake's Python logger. This will either be the Drake Logger's level or
    else the root Logger's level in case Drake Logger's level is NOTSET.

    This only syncs the "effective" level, not the "enabled for" level. The
    "enabled for" level would also incorporate the global "logging.disable"
    setting, but we do not reflect that back into C++ spdlog at the moment.
    """
    level = _drake_logger.level or _root_logger.level
    _set_log_level(_python_level_to_spdlog(level))


def _monkey_patch_logger_level_events():
    """Adds a hook into Drake's Python Logger object such that changes to
    Python logging levels are reflected back into Drake's C++ spdlog level.
    """
    # To notice log level changes (whether to the root logger or our logger) we
    # intercept calls to our logger's cache-clearing function. Specifically,
    # when a user calls `Logger.setLevel(...)`, the logger tells the manager
    # singleton to clear its cache ...
    #
    # https://github.com/python/cpython/blob/c2b57974/Lib/logging/__init__.py#L1457-L1462
    #
    # ... which asks each Logger in turn to clear its cache.
    #
    # https://github.com/python/cpython/blob/c2b57974/Lib/logging/__init__.py#L1412-L1423
    #
    # That notification is triggered for any logger level change, including
    # both the root logger and Drake's own logger. Therefore, syncing the C++
    # level based on cache-clearing events is sufficient to cover all cases.
    #
    # This relies on the implementation details of CPython's logging module,
    # so we might need to adapt this technique to track CPython changes. Our
    # CI testing will inform us in case this technique ever stops working.
    class spdlog_syncing_dict(dict):
        def clear(self):
            super().clear()
            _sync_spdlog_level()

    _drake_logger._cache = spdlog_syncing_dict()


# Reflect the Python level to C++ level iff the Python sink is fed by spdlog.
if getattr(_drake_logger, "_tied_to_spdlog", False):
    _monkey_patch_logger_level_events()


def configure_logging():
    """Convenience function that configures the root Python logging module in
    a tasteful way for Drake. Using this function is totally optional; there
    is no requirement to call it prior to using Drake or generating messages.
    We offer it as a convenience only because Python's logging defaults are
    more spartan than Drake's C++ logging format (e.g., Python does not show
    message timestamps by default).

    Note:
        pydrake logs using Python's built-in ``logging`` module. To access
        pydrake's ``logging.Logger``, use ``logging.getLogger("drake")``.
        You can configure log settings using that object whether or not you
        have called ``configure_logging()`` first.

    See also:
        https://docs.python.org/3/library/logging.html
    """
    format = "[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s"
    _logging.basicConfig(level=_logging.INFO, format=format)
    _drake_logger.setLevel(_logging.NOTSET)
    _logging.addLevelName(5, "TRACE")


def _wrap_to_match_input_shape(f):
    # See docstring for `WrapToMatchInputShape` in `eigen_pybind.h` for more
    # details.
    # N.B. We cannot use `inspect.Signature` due to the fact that pybind11's
    # instance method is not inspectable for overloads.
    assert callable(f), f

    @functools.wraps(f)
    def wrapper(self, *args, **kwargs):
        # Call the function first to permit it to raise the appropriate
        # TypeError from pybind11 if the inputs are not correctly formatted.
        out = f(self, *args, **kwargs)
        if isinstance(out, np.ndarray):
            arg_list = tuple(args) + tuple(kwargs.values())
            assert len(arg_list) == 1
            (arg,) = arg_list
            in_shape = np.asarray(arg).shape
            return out.reshape(in_shape)
        else:
            return out

    return wrapper


# (This constant relates to cpp_template.py, but for dependency reasons we need
# to lift it up into the common module.)
#
# When we need to create a well-formed Python class name that joins a template
# name with its template arguments, we will use this unicode character to
# partition the template name vs its arguments, as kind of "name mangling".
#
# For example, the template instantiation expression LeafSystem_[AutoDiffXd]
# refers to Python class named LeafSystem_𝓣AutoDiffXd. (We cannot use square
# brackets as a Python class names, or else we'd break many of Python's tools
# such as Mypy.)
#
# See _pretty_class_name() below for a demangling function to help display the
# template expression to the user, instead of the mangled name.
#
# This letter is 'U+1D4E3 MATHEMATICAL BOLD SCRIPT CAPITAL T'.
_UNICODE_TEMPLATE_OPENER = "𝓣"

# (This constant relates to cpp_template.py, but for dependency reasons we need
# to lift it up into the common module.)
#
# Similarly, when we need to mangle multiple template arguments into a Python
# class name, we will use this unicode character to separate the arguments.
#
# This letter is 'U+1D4FD MATHEMATICAL BOLD SCRIPT SMALL T'.
_UNICODE_TEMPLATE_PARAM_SPLIT = "𝓽"


def _pretty_class_name(cls: type, *, use_qualname: bool = False) -> str:
    """Given a class, returns its ``cls.__name__`` respelled to be suitable for
    display to a user, in particular by respelling C++ template arguments using
    their conventional ``FooBar_[AutoDiffXd]`` expression spelling instead of
    the mangled unicode name. Note that the returned name might not be a valid
    Python identifier, though it should still be a valid Python expression.

    If the class is not a template, simply returns ``cls.__name__`` unchanged.

    When ``use_qualname`` is true, uses ``cls.__qualname__`` instead of
    ``cls.__name__``.
    """
    if use_qualname:
        name = cls.__qualname__
    else:
        name = cls.__name__
    magic = _UNICODE_TEMPLATE_OPENER
    if magic not in name:
        return name
    name = name.replace(magic, "[") + "]"
    name = name.replace(_UNICODE_TEMPLATE_PARAM_SPLIT, ", ")
    return name
