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
            if t.__module__ not in ("builtins", "numpy"):
                return t
        # Unable to infer type.
        return None


def _check_returned_array_type(*, cls_name, y, expected_type):
    y = _np.asarray(y)
    if y.size == 0:
        return
    actual_type = _resolve_array_type(y)
    expected_name = expected_type.__name__
    if actual_type is None:
        raise TypeError(
            f"When {cls_name} is called with an array of type {expected_name} "
            f"the return value must be the same type."
        )
    if actual_type is not expected_type:
        actual_name = actual_type.__name__
        raise TypeError(
            f"When {cls_name} is called with an array of type {expected_name} "
            f"the return value must be the same type, not {actual_name}."
        )


def _check_array_shape(*, var_name, var, dim, size):
    try:
        var = _np.asarray(var)
        valid = True
    except Exception:
        valid = False
    if valid:
        if dim == 0:
            valid = var.ndim == 0
        else:
            valid = var.ndim == 1 or var.ndim == 2
    if valid:
        valid = var.size == size
    if not valid:
        ndim_hint = "0 (scalar)" if (dim == 0) else "1 or 2 (vector)"
        raise RuntimeError(
            f"{var_name} must be of .ndim = {ndim_hint} and .size = {size}. "
            f"Got .ndim = {var.ndim} and .size = {var.size} instead."
        )


def _wrap_user_func(
    cls_name, func, num_vars, num_outputs, output_dim, expected_type
):
    def _wrapped(x):
        _check_array_shape(
            var_name=f"{cls_name}: Input", var=x, dim=1, size=num_vars
        )
        y = func(x)
        _check_array_shape(
            var_name=f"{cls_name}: Return value",
            var=y,
            dim=output_dim,
            size=num_outputs,
        )
        _check_returned_array_type(
            cls_name=cls_name, y=y, expected_type=expected_type
        )
        return y

    return _wrapped
