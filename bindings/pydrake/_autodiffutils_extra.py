# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import numpy as np
from pydrake.common.deprecation import deprecated_callable


def InitializeAutoDiffTuple(*args):
    """Given a series of array_like input arguments, create a tuple of
    corresponding AutoDiff matrices with values equal to the input matrices
    and properly initialized derivative vectors.

    The size of the derivative vector of each element of the matrices in the
    output tuple will be the same, and will equal the sum of the number of
    elements of the matrices in args.

    The 0th element of the derivative vectors will correspond to the
    derivative with respect to the 0th element of the first argument.
    Subsequent derivative vector elements correspond first to subsequent
    elements of the first input argument (traversed first by row, then by
    column), and so on for subsequent arguments.

    This is a pythonic implementation of drake::math::InitializeAutoDiffTuple
    in C++.
    """

    num_derivatives = 0
    for arg in args:
        num_derivatives += np.asarray(arg).size

    autodiff_tuple = []
    deriv_num_start = 0
    for arg in args:
        autodiff_tuple.append(InitializeAutoDiff(arg,
                                                 num_derivatives,
                                                 deriv_num_start))
        deriv_num_start += np.asarray(arg).size

    return tuple(autodiff_tuple)


initializeAutoDiffTuple = deprecated_callable(
    "Use InitializeAutoDiffTuple()", date="2022-02-01"
)(InitializeAutoDiffTuple)


@np.vectorize
def autodiff_equal_to(a, b, *, semantic=False):
    """
    Provides a structural equality check for arrays of AutoDiffXd scalars, i.e.
    returns True if both the values and derivates are equal.

    Arguments:
        a, b:
            Arrays to compare.
        semantic:
            If False, performs *literal* comparison, meaning the value and
            derivatives must match in both value and shape.
            If True, performs *semantic* comparison, meaning that empty
            derivatives is equivalent to purely zero-valued derivatives.
            Note: Zero-valued derivatives of different size are *not*
            equivalent.
    """
    assert isinstance(a, AutoDiffXd), type(a)
    assert isinstance(b, AutoDiffXd), type(b)
    if a.value() == b.value():
        da = a.derivatives()
        db = b.derivatives()
        if da.shape == db.shape and (da == db).all():
            return True
        da_empty = da.size == 0
        db_empty = db.size == 0
        if semantic and (da_empty or db_empty):
            da_zero = (da == 0.0).all()
            db_zero = (db == 0.0).all()
            if (da_zero and db_empty) or (da_empty and db_zero):
                return True
    return False
