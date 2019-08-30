"""
Provides utilities to check if an object supports pickling (serlialization).
"""

from io import BytesIO
import pickle

import numpy as np
import six

import pydrake.common.test_utilities.numpy_compare as numpy_compare
from pydrake.symbolic import Expression

_PYBIND11_METACLASS = type(Expression)


def _assert_equal(test, a, b):
    if isinstance(a, np.ndarray):
        numpy_compare.assert_equal(a, b)
    else:
        test.assertEqual(a, b)


def assert_pickle(test, obj, value_to_compare=lambda x: x.__dict__, T=None):
    """
    Asserts that an object can be dumped and loaded and still maintain its
    value.

    Args:
        test: Instance of `unittest.TestCase` (for assertions).
        obj: Obj to dump and then load.
        value_to_compare: (optional) Value to extract from the object to
            compare. By default, compares dictionaries.
        T: (optional) When pickling template instantiations on scalar types,
            pass the scalar type T. This is used becaues `Expression` is
            currently not a serializable type.
    """
    metaclass = type(type(obj))
    # Pickling is explicitly disabled for our pybind11 types in Python 2.
    if six.PY2 and metaclass == _PYBIND11_METACLASS:
        with test.assertRaises(RuntimeError) as cm:
            pickle.dump(obj, BytesIO())
        test.assertIn(
            "Pickling in pydrake is disabled in Python 2", str(cm.exception))
    elif T == Expression:
        # Pickling not enabled for Expression.
        return
    else:
        f = BytesIO()
        pickle.dump(obj, f)
        f.seek(0)
        obj_again = pickle.load(f)
        _assert_equal(test, value_to_compare(obj), value_to_compare(obj_again))
