from __future__ import print_function
# N.B. We are purposely not importing `division` to test nominal Python2
# behavior.

import pydrake.autodiffutils as mut
from pydrake.autodiffutils import AutoDiffXd

import unittest
import numpy as np
import pydrake.math as drake_math

from pydrake.test.algebra_test_util import ScalarAlgebra, VectorizedAlgebra
from pydrake.test.autodiffutils_test_util import (
    autodiff_scalar_pass_through,
    autodiff_vector_pass_through,
)

# Use convenience abbreviation.
AD = AutoDiffXd


class TestAutoDiffXd(unittest.TestCase):
    def _check_scalar(self, actual, expected):
        if isinstance(actual, bool):
            self.assertTrue(isinstance(expected, bool))
            self.assertEqual(actual, expected)
        elif isinstance(actual, float):
            self.assertTrue(isinstance(expected, float))
            self.assertEqual(actual, expected)
        else:
            self.assertAlmostEqual(actual.value(), expected.value())
            self.assertTrue(
                (actual.derivatives() == expected.derivatives()).all(),
                (actual.derivatives(), expected.derivatives()))

    def _check_array(self, actual, expected):
        expected = np.array(expected)
        self.assertEqual(actual.dtype, expected.dtype)
        self.assertEqual(actual.shape, expected.shape)
        if actual.dtype == object:
            for a, b in zip(actual.flat, expected.flat):
                self._check_scalar(a, b)
        else:
            self.assertTrue((actual == expected).all())

    def test_scalar_api(self):
        a = AD(1, [1., 0])
        self.assertEqual(a.value(), 1.)
        self.assertTrue((a.derivatives() == [1., 0]).all())
        self.assertEqual(str(a), "AD{1.0, nderiv=2}")
        self.assertEqual(repr(a), "<AutoDiffXd 1.0 nderiv=2>")
        self._check_scalar(a, a)
        # Test construction from `float` and `int`.
        self._check_scalar(AD(1), AD(1., []))
        self._check_scalar(AD(1.), AD(1., []))
        # Test implicit conversion.
        self._check_scalar(
            autodiff_scalar_pass_through(1),  # int
            AD(1., []))
        self._check_scalar(
            autodiff_scalar_pass_through(1.),  # float
            AD(1., []))

    def test_array_api(self):
        a = AD(1, [1., 0])
        b = AD(2, [0, 1.])
        x = np.array([a, b])
        self.assertEqual(x.dtype, object)
        # Idempotent check.
        self._check_array(x, x)
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
        self._check_array(
            autodiff_vector_pass_through([1, 2]),  # int
            [AD(1., []), AD(2., [])])
        self._check_array(
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
        algebra.check_value(-a, AD(-1, [-1., 0]))
        algebra.check_value(a + b, AD(3, [1, 1]))
        algebra.check_value(a + 1, AD(2, [1, 0]))
        algebra.check_value(1 + a, AD(2, [1, 0]))
        algebra.check_value(a - b, AD(-1, [1, -1]))
        algebra.check_value(a - 1, AD(0, [1, 0]))
        algebra.check_value(1 - a, AD(0, [-1, 0]))
        algebra.check_value(a * b, AD(2, [2, 1]))
        algebra.check_value(a * 2, AD(2, [2, 0]))
        algebra.check_value(2 * a, AD(2, [2, 0]))
        algebra.check_value(a / b, AD(1./2, [1./2, -1./4]))
        algebra.check_value(a / 2, AD(0.5, [0.5, 0]))
        algebra.check_value(2 / a, AD(2, [-2, 0]))
        # Logical
        algebra.check_logical(lambda x, y: x == y, a, a, True)
        algebra.check_logical(lambda x, y: x != y, a, a, False)
        algebra.check_logical(lambda x, y: x < y, a, b, True)
        algebra.check_logical(lambda x, y: x <= y, a, b, True)
        algebra.check_logical(lambda x, y: x > y, a, b, False)
        algebra.check_logical(lambda x, y: x >= y, a, b, False)
        # Additional math
        # - See `math_overloads_test` for scalar overloads.
        algebra.check_value(a**2, AD(1, [2., 0]))
        algebra.check_value(algebra.log(a), AD(0, [1., 0]))
        algebra.check_value(algebra.abs(-a), AD(1, [1., 0]))
        algebra.check_value(algebra.exp(a), AD(np.e, [np.e, 0]))
        algebra.check_value(algebra.sqrt(a), AD(1, [0.5, 0]))
        algebra.check_value(algebra.pow(a, 2), AD(1, [2., 0]))
        algebra.check_value(algebra.sin(c), AD(0, [1, 0]))
        algebra.check_value(algebra.cos(c), AD(1, [0, 0]))
        algebra.check_value(algebra.tan(c), AD(0, [1, 0]))
        algebra.check_value(algebra.arcsin(c), AD(0, [1, 0]))
        algebra.check_value(algebra.arccos(c), AD(np.pi / 2, [-1, 0]))
        algebra.check_value(algebra.arctan2(c, d), AD(0, [1, 0]))
        algebra.check_value(algebra.sinh(c), AD(0, [1, 0]))
        algebra.check_value(algebra.cosh(c), AD(1, [0, 0]))
        algebra.check_value(algebra.tanh(c), AD(0, [1, 0]))
        algebra.check_value(algebra.min(a, b), a_scalar)
        algebra.check_value(algebra.max(a, b), b_scalar)
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
        algebra.check_value(ceil_a, a_scalar.value())
        algebra.check_value(floor_a, a_scalar.value())
        # Return value so it can be inspected.
        return a

    def test_scalar_algebra(self):
        a = self._check_algebra(
            ScalarAlgebra(
                self._check_scalar, scalar_to_float=lambda x: x.value()))
        self.assertEqual(type(a), AD)

    def test_array_algebra(self):
        a = self._check_algebra(
            VectorizedAlgebra(
                self._check_array,
                scalar_to_float=lambda x: x.value()))
        self.assertEqual(type(a), np.ndarray)
        self.assertEqual(a.shape, (2,))
