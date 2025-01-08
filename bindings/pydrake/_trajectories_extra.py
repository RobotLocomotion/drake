from pydrake.common import (
    _MangledName,
    pretty_class_name as _pretty_class_name,
)


def __getattr__(name):
    """Rewrites requests for Foo[bar] into their mangled form, for backwards
    compatibility with unpickling.
    """
    return _MangledName.module_getattr(
        module_name=__name__, module_globals=globals(), name=name)


def _wrapped_trajectory_repr(wrapped_trajectory):
    cls = type(wrapped_trajectory)
    return f"{_pretty_class_name(cls)}({wrapped_trajectory.unwrap()!r})"


def _add_repr_functions():
    for param in _WrappedTrajectory_.param_list:
        cls = _WrappedTrajectory_[param]
        setattr(cls, "__repr__", _wrapped_trajectory_repr)


_add_repr_functions()
