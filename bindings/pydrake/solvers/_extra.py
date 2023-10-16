import numpy as _np


def _resolve_array_type(x):
    # Resolves the scalar type for a given array.
    assert isinstance(x, _np.ndarray), type(x)
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


def _check_returned_array_type(cls_name, y, expected_type):
    # Used by CheckReturnedArrayType in C++.
    if y.size == 0:
        return
    actual_type = _resolve_array_type(y)
    expected_name = expected_type.__name__
    if actual_type is None:
        raise TypeError(
            f"When {cls_name} is called with an array of type {expected_name} "
            f"the return value must be the same type.")
    if actual_type is not expected_type:
        actual_name = actual_type.__name__
        raise TypeError(
            f"When {cls_name} is called with an array of type {expected_name} "
            f"the return value must be the same type, not {actual_name}.")
