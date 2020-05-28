# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import numpy as np

from pydrake.common.cpp_param import List, Vector4d


def _is_maybe_vector4d(value):
    if isinstance(value, (list, np.ndarray)) and len(value) == 4:
        value = np.asarray(value)
        if value.dtype == float:
            return True
    return False


def _AbstractValue_Make(value):
    """Returns an AbstractValue containing the given ``value``."""
    if _is_maybe_vector4d(value):
        cls = Vector4d
    elif isinstance(value, list) and len(value) > 0:
        inner_cls = type(value[0])
        cls = List[inner_cls]
    else:
        cls = type(value)
    value_cls, _ = Value.get_instantiation(cls, throw_error=False)
    if value_cls is None:
        value_cls = Value[object]
    return value_cls(value)


AbstractValue.Make = _AbstractValue_Make
