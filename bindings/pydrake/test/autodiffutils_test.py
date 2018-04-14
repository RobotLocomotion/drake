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
    AutoDiffContainer,
    autodiff_increment,
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
        if actual.dtype == object or actual.dtype == AD:
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
        # Test multi-element pass-through.
        x = np.array([AD(1.), AD(2.), AD(3.)])
        self._check_array(autodiff_vector_pass_through(x), x)
        # Ensure fixed-size vectors are correctly converted (#9886).
        self._check_array(autodiff_vector3_pass_through(x), x)
        # Ensure we can copy.
        self._check_scalar(copy.copy(a), a)
        self._check_scalar(copy.deepcopy(a), a)

    def test_array_api(self):
        a = AD(1, [1., 0])
        b = AD(2, [0, 1.])
        x = np.array([a, b])
        self.assertEqual(x.dtype, AD)
        # Idempotent check.
        self._check_array(x, x)

    def test_array_casting(self):
        a = AD(1, [1., 0])
        b = AD(2, [0, 1.])
        x = np.array([a, b])
        # Explicit casting using `astype`.
        xf = x.astype(dtype=np.float)
        self._check_array(xf, [1., 2])
        x0 = np.zeros((3, 3), dtype=AD)
        self.assertTrue(isinstance(x0[0, 0], AD))
        xI = np.eye(3).astype(AD)
        self.assertTrue(isinstance(xI[0, 0], AD))

        # Promotion (upcasting? downcasting?) is implicitly castable.
        x[0] = 0
        x[0] = 0.
        x[0] = False
        # Assign via slicing with implicit casting.
        # N.B. This requires numpy/numpy#11076.
        x[:] = 0
        x[:] = 0.
        x[:] = False

        # Assigning via slicing is an explicit cast.
        xf = np.zeros(2, dtype=np.float)
        xf[:] = x
        # Assigning via an element is an implicit cast.
        with self.assertRaises(TypeError):
            xf[0] = x[0]
        # Try `int`; we do not have an explicit caster registered.
        xi = np.zeros(2, dtype=np.int)
        with self.assertRaises(ValueError):
            xi[:] = x
        # TODO(eric.cousineau): Fix this.
        with self.assertRaises(TypeError):
            xi[0] = x[0]
        # Converts.
        x = np.zeros((3, 3), dtype=AD)
        self.assertTrue(isinstance(x[0, 0], AD))
        x = np.eye(3).astype(AD)
        self.assertTrue(isinstance(x[0, 0], AD))

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
        algebra.check_logical(algebra.eq, a, a, True)
        algebra.check_logical(lambda x, y: x != y, a, a, False)
        algebra.check_logical(algebra.ne, a, a, False)
        algebra.check_logical(lambda x, y: x < y, a, b, True)
        algebra.check_logical(algebra.lt, a, b, True)
        algebra.check_logical(lambda x, y: x <= y, a, b, True)
        algebra.check_logical(algebra.le, a, b, True)
        algebra.check_logical(lambda x, y: x > y, a, b, False)
        algebra.check_logical(algebra.gt, a, b, False)
        algebra.check_logical(lambda x, y: x >= y, a, b, False)
        algebra.check_logical(algebra.ge, a, b, False)
        # Additional math
        # - See `math_overloads_test` for scalar overloads.
        algebra.check_value(a**2, AD(1, [2., 0]))
        algebra.check_value(algebra.log(a), AD(0, [1., 0]))
        algebra.check_value(algebra.abs(-a), AD(1, [1., 0]))
        algebra.check_value(algebra.exp(a), AD(np.e, [np.e, 0]))
        algebra.check_value(algebra.sqrt(a), AD(1, [0.5, 0]))
        algebra.check_value(algebra.pow(a, 2), AD(1, [2., 0]))
        algebra.check_value(algebra.pow(a, 0.5), AD(1, [0.5, 0]))
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
        algebra.check_value(algebra.ceil(a), a_scalar.value())
        algebra.check_value(algebra.floor(a), a_scalar.value())
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

    def test_linear_algebra(self):
        a_scalar = AD(1, [1., 0])
        b_scalar = AD(2, [0, 1.])
        A = np.array([[a_scalar, a_scalar]])
        B = np.array([[b_scalar, b_scalar]]).T
        C = np.dot(A, B)
        self._check_array(C, [[AD(4, [4., 2])]])

        # `matmul` not supported for `dtype=object` (#11332). `np.dot` should
        # be used instead.
        with self.assertRaises(TypeError):
            C2 = np.matmul(A, B)

        # Type mixing
        Bf = np.array([[2., 2]]).T
        C2 = np.dot(A, Bf)  # Leverages implicit casting.
        self._check_array(C2, [[AD(4, [4., 0])]])

        # Other methods.
        X = np.array([[a_scalar, b_scalar], [b_scalar, a_scalar]])
        self._check_scalar(np.trace(X), AD(2, [2., 0]))

        # `inv` is a ufunc that we must implement, if possible. However, given
        # that this is currently `dtype=object`, it would be extremely unwise
        # to do so. See #8116 for alternative.
        with self.assertRaises(TypeError):
            Y = np.linalg.inv(X)

        to_value = np.vectorize(AutoDiffXd.value)
        # Use workaround for inverse. For now, just check values.
        X_float = to_value(X)
        Xinv_float = np.linalg.inv(X_float)
        Xinv = drake_math.inv(X)
        np.testing.assert_equal(to_value(Xinv), Xinv_float)

    def test_array_reference(self):
        # Test referencing from Python to C++.
        x = np.array([
            AD(1, [1., 0]),
            AD(2, [0, 1.])])
        autodiff_increment(x)
        self._check_array(x, [AD(2, [1., 0]), AD(3, [0, 1.])])
        # Test referencing from C++ to Python.
        c = AutoDiffContainer()
        self._check_array(c.value(), [[AD(10, [1., 0]), AD(100, [0, 1.])]])
        xc = c.value()
        xc *= 2
        self._check_array(c.value(), [[AD(20, [2., 0]), AD(200, [0, 2.])]])
