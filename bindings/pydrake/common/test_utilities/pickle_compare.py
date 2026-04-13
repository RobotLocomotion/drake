"""
Provides utilities to check if an object supports pickling (serlialization).
"""

import pickle

import numpy as np

import pydrake.common.test_utilities.numpy_compare as numpy_compare
from pydrake.math import (
    BsplineBasis_,
    RigidTransform_,
    RollPitchYaw_,
    RotationMatrix_,
)
from pydrake.symbolic import (
    Expression,
)

_PYBIND11_METACLASS = type(Expression)


def _assert_equal(test, a, b):
    if isinstance(a, np.ndarray):
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
        T: (optional) When pickling template instantiations on scalar types,
            pass the scalar type T.
    """
    obj_again = pickle.loads(pickle.dumps(obj))
    if T == Expression:
        if isinstance(obj, Expression):
            test.assertTrue(obj.EqualTo(obj_again))
        elif isinstance(
            obj, (RotationMatrix_[Expression], RigidTransform_[Expression])
        ):
            test.assertTrue(obj.IsExactlyEqualTo(obj_again).Evaluate())
        elif isinstance(obj, RollPitchYaw_[Expression]):
            test.assertTrue(
                obj.IsNearlyEqualTo(other=obj_again, tolerance=0).Evaluate()
            )
        elif isinstance(obj, BsplineBasis_[Expression]):
            test.assertTrue((obj == obj_again).Evaluate())
    else:
        _assert_equal(test, value_to_compare(obj), value_to_compare(obj_again))
