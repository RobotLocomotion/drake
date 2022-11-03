# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

from pydrake.common.cpp_param import List

_virtual_types = []


def _register_virtual_value_type(cls):
    print(f"yar: {cls}")
    _virtual_types.append(cls)


def _AbstractValue_Make(value):
    """Returns an AbstractValue containing the given ``value``."""
    if isinstance(value, list) and len(value) > 0:
        inner_cls = type(value[0])
        cls = List[inner_cls]
    else:
        cls = type(value)
    if isinstance(cls, type):
        for maybe_parent_cls in _virtual_types:
            if issubclass(cls, maybe_parent_cls):
                cls = maybe_parent_cls
                break
    value_cls, _ = Value.get_instantiation(cls, throw_error=False)
    if value_cls is None:
        value_cls = Value[object]
    return value_cls(value)


AbstractValue.Make = _AbstractValue_Make
