import numpy as _np


def _scalar_types_match(actual_type, expected_type):
    return (actual_type is expected_type) or (
        (expected_type is float) and (actual_type in (_np.float32, _np.float64))
    )


def _wrap_user_evaluator_func(
    cls_name: str,
    user_evaluator_func,
    num_vars: int,
    num_outputs: int,
    output_dim: int,
    expected_type: type,
):
    """The implementation of the C++ helper `WrapUserEvaluatorFunc`."""
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
            if not _scalar_types_match(type(y), expected_type):
                raise TypeError(
                    f"When {cls_name} is called with an array of type "
                    f"{expected_type.__name__} the return value must be a "
                    "scalar (not array) of the same type, not a "
                    f"{type(y).__name__} ({y!r})."
                )
            return y

        # For constraints, the return value should be a vector of the expected
        # type (either an np.ndarray or convertible to it).
        if y is None:
            raise TypeError(f"{cls_name} returned None")
        try:
            y = _np.asarray(y)
        except Exception as e:
            raise TypeError(
                f"When {cls_name} is called with an array of type "
                f"{expected_type.__name__} the return value must be an array "
                f"of the same type (numpy conversion error: {e})."
            )

        # Validate the shape.
        valid = (y.size == num_outputs) and (
            y.ndim == 1 or (y.ndim == 2 and 1 in y.shape)
        )
        if not valid:
            raise ValueError(
                f"{cls_name} return value must be array of "
                f".ndim = 1 or 2 (vector) and .size = {num_outputs}. "
                f"Got .ndim = {y.ndim} and .size = {y.size} instead."
            )
        if num_outputs == 0:
            return y

        # Validate the dtype.
        if y.dtype != object:
            if y.dtype is float or y.dtype is int:
                actual_type = float
            else:
                actual_type = y.dtype.type
        else:
            # Search array for any non-builtin and non-numpy types.
            for yi in y.flat:
                t = type(yi)
                if t.__module__ not in ("builtins", "numpy"):
                    actual_type = t
                    break
            else:
                # Unable to infer type.
                actual_type = type(None)
        if not _scalar_types_match(actual_type, expected_type):
            raise TypeError(
                f"When {cls_name} is called with an array of type "
                f"{expected_type.__name__} the return value must be the same "
                f"type, not {actual_type.__name__}."
            )

        return y

    return _wrapped
