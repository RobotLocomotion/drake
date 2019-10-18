# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import numpy as np


def initializeAutoDiffTuple(*args):
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

    This is a pythonic implementation of drake::math::initializeAutoDiffTuple
    in C++.
    """

    num_derivatives = 0
    for arg in args:
        num_derivatives += np.asarray(arg).size

    autodiff_tuple = []
    deriv_num_start = 0
    for arg in args:
        autodiff_tuple.append(initializeAutoDiff(arg,
                                                 num_derivatives,
                                                 deriv_num_start))
        deriv_num_start += np.asarray(arg).size

    return tuple(autodiff_tuple)
