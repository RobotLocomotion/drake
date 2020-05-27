# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import numpy as np


def _resolve_array_type(x):
    # Resolves the scalar type for a given array.
    assert isinstance(x, np.ndarray), type(x)
    assert x.size != 0
    if x.dtype != object:
        if x.dtype == float or x.dtype == int:
            return float
        else:
            return x.dtype.type
    else:
        # Search array for any non-builtin and non-numpy types.
        for xi in x.flat:
            t = type(xi)
            if t.__module__ not in ('builtins', 'numpy'):
                return t
        # Unable to infer type.
        return None


def _check_array_type(var_name, x, expected_type):
    # Used by CheckArrayType in C++.
    if x.size == 0:
        return
    actual_type = _resolve_array_type(x)
    expected_name = expected_type.__name__
    if actual_type is None:
        raise RuntimeError(
            f"{var_name} must be of scalar type {expected_name}, but unable "
            f"to infer scalar type.")
    if actual_type is not expected_type:
        actual_name = actual_type.__name__
        raise RuntimeError(
            f"{var_name} must be of scalar type {expected_name}. Got "
            f"{actual_name} instead.")
