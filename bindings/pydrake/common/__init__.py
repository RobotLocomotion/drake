import collections
import functools
import inspect

import numpy as np

from ._module_py import *

# When running from python, turn DRAKE_ASSERT and DRAKE_DEMAND failures into
# SystemExit, instead of process aborts.  See RobotLocomotion/drake#5268.
set_assertion_failure_to_throw_exception()


def _wrap_to_match_input_shape(f):
    # See docstring for `WrapToMatchInputShape` in `eigen_pybind.h` for more
    # details.
    # N.B. We cannot use `inspect.Signature` due to the fact that pybind11's
    # instance method is not inspectable for overloads.
    assert callable(f), f

    @functools.wraps(f)
    def wrapper(self, *args, **kwargs):
        # Call the function first to permit it to raise the appropriate
        # TypeError from pybind11 if the inputs are not correctly formatted.
        out = f(self, *args, **kwargs)
        if isinstance(out, np.ndarray):
            arg_list = tuple(args) + tuple(kwargs.values())
            assert len(arg_list) == 1
            (arg,) = arg_list
            in_shape = np.asarray(arg).shape
            return out.reshape(in_shape)
        else:
            return out

    return wrapper
