from __future__ import absolute_import, print_function

import numpy as np

from ._pydrake_autodiffutils import *


def wrap(wrapper_type, x):
    print("wrapping:", x)
    wrapped = wrapper_type(x.shape)
    for i in range(x.size):
        wrapped[i] = x.flat[i]
    print("returning:", wrapped)
    return wrapped


def unwrap(wrapped):
    unwrapped = np.empty(wrapped.size(), dtype=AutoDiffXd)
    for i in range(wrapped.size()):
        unwrapped.flat[i] = wrapped[i]
    return unwrapped
