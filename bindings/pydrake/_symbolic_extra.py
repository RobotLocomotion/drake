# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.
import functools

# Import aliases.
# TODO(eric.cousineau): Deprecate, then remove these in lieu of `np.{func}`
from pydrake.math import (
    log,
    abs,
    exp,
    pow,
    sqrt,
    sin,
    cos,
    tan,
    asin,
    acos,
    atan,
    atan2,
    sinh,
    cosh,
    tanh,
    min,
    max,
    ceil,
    floor
)


def logical_and(*formulas):
    assert len(formulas) >= 1, "Must supply at least one operand"
    return functools.reduce(__logical_and, formulas)


def logical_or(*formulas):
    assert len(formulas) >= 1, "Must supply at least one operand"
    return functools.reduce(__logical_or, formulas)
