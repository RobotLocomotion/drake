import numpy as np

from pydrake.common.cpp_template import TemplateClass, _get_module_from_stack


def _resolve_dtype_type(x):
    if x.dtype == object:
        assert x.size > 0, x
        return type(x.flat[0])
    return x.dtype.type


def _AbstractValue_Make(value):
    """Returns an AbstractValue containing the given ``value``."""
    if isinstance(value, np.ndarray):
        T = _resolve_dtype_type(value)
        if value.ndim == 1:
            cls = VectorX[T]
        elif value.ndim == 2:
            cls = MatrixX[T]
        else:
            raise ValueError("Only ndarray's of dimension 1 or 2 allowed")
    else:
        cls = type(value)
    value_cls, _ = Value.get_instantiation(cls, throw_error=False)
    if value_cls is None:
        value_cls = Value[object]
    return value_cls(value)


AbstractValue.Make = _AbstractValue_Make


def _create_placeholder_cls(template, param, doc=""):
    """Creates and registers a placeholder class in a Template."""

    class Placeholder:
        def __init__(self):
            raise NotImplementedError()

    template.add_instantiation(param, Placeholder)
    Placeholder.__doc__ = doc
    return Placeholder


VectorX = TemplateClass("VectorX")
MatrixX = TemplateClass("MatrixX")

_bind_all_eigen_value_instantiations()
