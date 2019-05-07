from __future__ import print_function
# N.B. We are purposely not importing `division` to test nominal Python2
# behavior.

import pydrake.autodiffutils as mut
from pydrake.autodiffutils import AutoDiffXd

import copy
import unittest

import numpy as np
import pydrake.math as drake_math

from pydrake.test.algebra_test_util import ScalarAlgebra, VectorizedAlgebra
from pydrake.test.autodiffutils_test_util import (
    autodiff_scalar_pass_through,
    autodiff_vector_pass_through,
    autodiff_vector3_pass_through,
)
import pydrake.common.test_utilities.numpy_compare as npc

# Use convenience abbreviation.
AD = AutoDiffXd


def check_logical(func, a, b, expected):
    # Checks logical operations, with broadcasting, checking that `a` and `b`
    # (of type `T`) have compatible logical operators when the left or right
    # operands are `float`s. Specifically, tests:
    # - f(T, T)
    # - f(T, float)
    # - f(float, T)
    npc.assert_equal(func(a, b), expected)
    af = npc.to_float(a)
    bf = npc.to_float(b)
    npc.assert_equal(func(a, bf), expected)
    npc.assert_equal(func(af, b), expected)


class TestAutoDiffXd(unittest.TestCase):
    def test_scalar_api(self):
        a = AD(1, [1., 0])
        self.assertEqual(a.value(), 1.)
        npc.assert_equal(a.derivatives(), [1., 0])
        self.assertEqual(str(a), "AD{1.0, nderiv=2}")
        self.assertEqual(repr(a), "<AutoDiffXd 1.0 nderiv=2>")
        npc.assert_equal(a, a)
        # Test construction from `float` and `int`.
        npc.assert_equal(AD(1), AD(1., []))
        npc.assert_equal(AD(1.), AD(1., []))
        # Test implicit conversion.
        npc.assert_equal(
            autodiff_scalar_pass_through(1),  # int
            AD(1., []))
        npc.assert_equal(
            autodiff_scalar_pass_through(1.),  # float
            AD(1., []))
        # Test multi-element pass-through.
        x = np.array([AD(1.), AD(2.), AD(3.)])
        npc.assert_equal(autodiff_vector_pass_through(x), x)
        # Ensure fixed-size vectors are correctly converted (#9886).
        npc.assert_equal(autodiff_vector3_pass_through(x), x)
        # Ensure we can copy.
        npc.assert_equal(copy.copy(a), a)
        npc.assert_equal(copy.deepcopy(a), a)

    def test_array_api(self):
        a = AD(1, [1., 0])
        b = AD(2, [0, 1.])
        x = np.array([a, b])
        self.assertEqual(x.dtype, object)
        # Idempotent check.
        npc.assert_equal(x, x)
        # Conversion.
        with self.assertRaises(TypeError):
            # Avoid implicit coercion, as this will imply information loss.
            xf = np.zeros(2, dtype=np.float)
            xf[:] = x
        with self.assertRaises(TypeError):
            # We could define `__float__` to allow this, but then that will
            # enable implicit coercion, which we should avoid.
            xf = x.astype(dtype=np.float)
        # Presently, does not convert.
        x = np.zeros((3, 3), dtype=AD)
        self.assertFalse(isinstance(x[0, 0], AD))
        x = np.eye(3).astype(AD)
        self.assertFalse(isinstance(x[0, 0], AD))
        # Test implicit conversion.
        npc.assert_equal(
            autodiff_vector_pass_through([1, 2]),  # int
            [AD(1., []), AD(2., [])])
        npc.assert_equal(
            autodiff_vector_pass_through([1., 2.]),  # float
            [AD(1., []), AD(2., [])])

    def _check_algebra(self, algebra):
        a_scalar = AD(1, [1., 0])
        b_scalar = AD(2, [0, 1.])
        c_scalar = AD(0, [1., 0])
        d_scalar = AD(1, [0, 1.])
        a, b, c, d = map(
            algebra.to_algebra, (a_scalar, b_scalar, c_scalar, d_scalar))

        # Arithmetic
        npc.assert_equal(-a, AD(-1, [-1., 0]))
        npc.assert_equal(a + b, AD(3, [1, 1]))
        npc.assert_equal(a + 1, AD(2, [1, 0]))
        npc.assert_equal(1 + a, AD(2, [1, 0]))
        npc.assert_equal(a - b, AD(-1, [1, -1]))
        npc.assert_equal(a - 1, AD(0, [1, 0]))
        npc.assert_equal(1 - a, AD(0, [-1, 0]))
        npc.assert_equal(a * b, AD(2, [2, 1]))
        npc.assert_equal(a * 2, AD(2, [2, 0]))
        npc.assert_equal(2 * a, AD(2, [2, 0]))
        npc.assert_equal(a / b, AD(1./2, [1./2, -1./4]))
        npc.assert_equal(a / 2, AD(0.5, [0.5, 0]))
        npc.assert_equal(2 / a, AD(2, [-2, 0]))
        # Logical
        check_logical(lambda x, y: x == y, a, a, True)
        check_logical(algebra.eq, a, a, True)
        check_logical(lambda x, y: x != y, a, a, False)
        check_logical(algebra.ne, a, a, False)
        check_logical(lambda x, y: x < y, a, b, True)
        check_logical(algebra.lt, a, b, True)
        check_logical(lambda x, y: x <= y, a, b, True)
        check_logical(algebra.le, a, b, True)
        check_logical(lambda x, y: x > y, a, b, False)
        check_logical(algebra.gt, a, b, False)
        check_logical(lambda x, y: x >= y, a, b, False)
        check_logical(algebra.ge, a, b, False)
        # Additional math
        # - See `math_overloads_test` for scalar overloads.
        npc.assert_equal(a**2, AD(1, [2., 0]))
        npc.assert_equal(algebra.log(a), AD(0, [1., 0]))
        npc.assert_equal(algebra.abs(-a), AD(1, [1., 0]))
        npc.assert_equal(algebra.exp(a), AD(np.e, [np.e, 0]))
        npc.assert_equal(algebra.sqrt(a), AD(1, [0.5, 0]))
        npc.assert_equal(algebra.pow(a, 2), AD(1, [2., 0]))
        npc.assert_equal(algebra.pow(a, 0.5), AD(1, [0.5, 0]))
        npc.assert_equal(algebra.sin(c), AD(0, [1, 0]))
        npc.assert_equal(algebra.cos(c), AD(1, [0, 0]))
        npc.assert_equal(algebra.tan(c), AD(0, [1, 0]))
        npc.assert_equal(algebra.arcsin(c), AD(0, [1, 0]))
        npc.assert_equal(algebra.arccos(c), AD(np.pi / 2, [-1, 0]))
        npc.assert_equal(algebra.arctan2(c, d), AD(0, [1, 0]))
        npc.assert_equal(algebra.sinh(c), AD(0, [1, 0]))
        npc.assert_equal(algebra.cosh(c), AD(1, [0, 0]))
        npc.assert_equal(algebra.tanh(c), AD(0, [1, 0]))
        npc.assert_equal(algebra.min(a, b), a_scalar)
        npc.assert_equal(algebra.max(a, b), b_scalar)
        # Because `ceil` and `floor` return `double`, we have to special case
        # this comparison since the matrix is `dtype=object`, even though the
        # elements are all doubles. We must cast it to float.
        # N.B. This would be fixed if we registered a UFunc for these
        # methods, so NumPy would have already returned a `float` array.
        ceil_a = algebra.ceil(a)
        floor_a = algebra.floor(a)
        if isinstance(algebra, VectorizedAlgebra):
            self.assertEqual(ceil_a.dtype, object)
            self.assertIsInstance(ceil_a[0], float)
            ceil_a = ceil_a.astype(float)
            floor_a = floor_a.astype(float)
        npc.assert_equal(ceil_a, a_scalar.value())
        npc.assert_equal(floor_a, a_scalar.value())
        # Return value so it can be inspected.
        return a

    def test_scalar_algebra(self):
        a = self._check_algebra(ScalarAlgebra())
        self.assertEqual(type(a), AD)

    def test_array_algebra(self):
        a = self._check_algebra(VectorizedAlgebra())
        self.assertEqual(type(a), np.ndarray)
        self.assertEqual(a.shape, (2,))

    def test_linear_algebra(self):
        a_scalar = AD(1, [1., 0])
        b_scalar = AD(2, [0, 1.])
        A = np.array([[a_scalar, a_scalar]])
        B = np.array([[b_scalar, b_scalar]]).T
        C = np.dot(A, B)
        npc.assert_equal(C, [[AD(4, [4., 2])]])

        # `matmul` not supported for `dtype=object` (#11332). `np.dot` should
        # be used instead.
        with self.assertRaises(TypeError):
            C2 = np.matmul(A, B)

        # Type mixing
        Bf = np.array([[2., 2]]).T
        C2 = np.dot(A, Bf)  # Leverages implicit casting.
        npc.assert_equal(C2, [[AD(4, [4., 0])]])

        # Other methods.
        X = np.array([[a_scalar, b_scalar], [b_scalar, a_scalar]])
        npc.assert_equal(np.trace(X), AD(2, [2., 0]))

        # `inv` is a ufunc that we must implement, if possible. However, given
        # that this is currently `dtype=object`, it would be extremely unwise
        # to do so. See #8116 for alternative.
        with self.assertRaises(TypeError):
            Y = np.linalg.inv(X)

        # Use workaround for inverse. For now, just check values.
        X_float = npc.to_float(X)
        Xinv_float = np.linalg.inv(X_float)
        Xinv = drake_math.inv(X)
        np.testing.assert_equal(npc.to_float(Xinv), Xinv_float)
