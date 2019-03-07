# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.
import functools


def logical_and(*formulas):
    assert len(formulas) >= 1, "Must supply at least one operand"
    return functools.reduce(__logical_and, formulas)


def logical_or(*formulas):
    assert len(formulas) >= 1, "Must supply at least one operand"
    return functools.reduce(__logical_or, formulas)
