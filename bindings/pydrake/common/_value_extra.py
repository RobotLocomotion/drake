import typing


def _AbstractValue_Make(value):
    """Returns an AbstractValue containing the given ``value``."""
    if isinstance(value, list) and len(value) > 0:
        cls = typing.List[type(value[0])]
    else:
        cls = type(value)
    value_cls, _ = Value.get_instantiation(cls, throw_error=False)
    if value_cls is None:
        value_cls = Value[object]
    return value_cls(value)


AbstractValue.Make = _AbstractValue_Make
