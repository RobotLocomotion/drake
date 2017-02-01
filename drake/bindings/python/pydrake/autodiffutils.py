from __future__ import absolute_import, print_function

import numpy as np

from ._pydrake_autodiffutils import *


def wrap(wrapper_type, x):
    """
    Construct an instance of the given opaque C++ wrapper type from
    input x, which should be a numpy ndarray. For example, to wrap
    a vector of autodiff scalars, you would do:

        wrap(autodiffutils.VectorXAutoDiffXd, x)

    """
    wrapped = wrapper_type(x.shape)
    for i in range(x.size):
        wrapped[i] = x.flat[i]
    return wrapped


def unwrap(wrapped):
    """
    Given an opaque C++ wrapper type, copy its contents into a new
    numpy ndarray of wrapped scalars
    """
    unwrapped = np.empty(wrapped.size(), dtype=AutoDiffXd)
    for i in range(wrapped.size()):
        unwrapped.flat[i] = wrapped[i]
    return unwrapped
