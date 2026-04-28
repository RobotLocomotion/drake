"""
Provides utilities to check if an object supports pickling (serlialization).
"""

import pickle

import numpy as np

import pydrake.common.test_utilities.numpy_compare as numpy_compare


def _assert_equal(test, a, b):
    if isinstance(a, np.ndarray) or isinstance(a, list):
        numpy_compare.assert_equal(a, b)
    elif isinstance(a, dict):
        test.assertEqual(a.keys(), b.keys())
        for key in a:
            _assert_equal(test, a[key], b[key])
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
        T: Vestigial and ignored.
    """
    # TODO(jwnimmer-tri) Remove unused T argument.
    obj_again = pickle.loads(pickle.dumps(obj))
    _assert_equal(test, value_to_compare(obj), value_to_compare(obj_again))
