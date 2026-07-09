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


def _wrap_user_evaluator_func(
    cls_name: str,
    user_evaluator_func,
    num_vars: int,
    num_outputs: int,
    output_dim: int,
    expected_type: type,
):
    assert output_dim in {0, 1}
    if output_dim == 0:
        assert num_outputs == 1

    def _wrapped(x):
        # This assertion is guaranteed by the C++ signature that calls us.
        assert x.ndim == 1

        # Check that the user passed the correct number of variables to the
        # evaluator. When called from MathematicalProgram this is guaranteed,
        # but direct calls to the evaluator might still have a mismatch.
        #
        # N.B. The C++ Eval code that calls us (via DoEval) in will (in Debug
        # builds) assertion-check the size, so this Python check is only
        # reachable in Release builds.
        if x.size != num_vars:
            raise ValueError(
                f"{cls_name} input must have .size = {num_vars}. "
                f"Got .size = {x.size} instead."
            )

        # Call the user's evaluator.
        y = user_evaluator_func(x)

        # For costs, the return value should be a scalar of the expected type.
        if output_dim == 0:
            actual_type = type(y)
            type_match = (actual_type == expected_type) or (
                (expected_type is float)
                and (actual_type in {_np.float32, _np.float64})
            )
            if not type_match:
                raise TypeError(
                    f"When {cls_name} is called with an array of type "
                    f"{expected_type.__name__} the return value must be a "
                    "scalar (not array) of the same type, not a "
                    f"{type(y).__name__} ({y!r})."
                )
            return y

        # For constraints, the return value should be a vector of the expected
        # type (either an np.ndarray or convertable to it).
        if y is None:
            raise TypeError(f"{cls_name} returned None")
        y_is_array = False
        try:
            y = _np.asarray(y)
            y_is_array = True
        except Exception:
            pass
        if not y_is_array:
            raise TypeError(
                f"When {cls_name} is called with an array of type "
                f"{expected_type.__name__} the return value must be an array "
                f"of the same type, not a {type(y).__name__} ({y!r})."
            )
        valid = (y.size == num_outputs) and (
            y.ndim == 1 or (y.ndim == 2 and 1 in y.shape)
        )
        if not valid:
            raise ValueError(
                f"{cls_name} return value must be array of "
                f".ndim = 1 or 2 (vector) and .size = {num_outputs}. "
                f"Got .ndim = {y.ndim} and .size = {y.size} instead."
            )
        _check_returned_array_type(
            cls_name=cls_name, y=y, expected_type=expected_type
        )
        return y

    return _wrapped
