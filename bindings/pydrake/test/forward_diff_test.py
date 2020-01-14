import unittest
import numpy as np
from pydrake.forwarddiff import sin, cos, derivative, gradient, jacobian
from pydrake.common.test_utilities import numpy_compare


class TestForwardDiff(unittest.TestCase):
    def test_trig(self):
        x = 0.5
        self.assertAlmostEqual(derivative(sin, x), np.cos(x))
        self.assertAlmostEqual(derivative(cos, x), -np.sin(x))

    def test_mul(self):
        x = 2.0
        self.assertAlmostEqual(derivative(lambda x: x * x, x),
                               2 * x)
        self.assertAlmostEqual(derivative(lambda x: 5 * x, x),
                               5)
        self.assertAlmostEqual(derivative(lambda x: x * 5, x),
                               5)

    def test_add(self):
        x = 2.0
        self.assertAlmostEqual(derivative(lambda x: x + x, x),
                               2)
        self.assertAlmostEqual(derivative(lambda x: x + 10, x),
                               1)
        self.assertAlmostEqual(derivative(lambda x: 10 + x, x),
                               1)

    def test_sub(self):
        x = 2.0
        self.assertAlmostEqual(derivative(lambda x: x - x, x),
                               0)
        self.assertAlmostEqual(derivative(lambda x: x - 11, x),
                               1)
        self.assertAlmostEqual(derivative(lambda x: 11 - x, x),
                               -1)

    def test_truediv(self):
        x = 2.0
        self.assertAlmostEqual(derivative(lambda x: x / x, x),
                               0)
        self.assertAlmostEqual(derivative(lambda x: x / 1.0, x),
                               1)
        self.assertAlmostEqual(derivative(lambda x: 1.0 / x, x),
                               -1.0 / x**2)

    def test_gradient_and_jacobian(self):
        # Explicitly test with type(x) == list, so that we ensure the API
        # handles this case.
        x = [1., 2.]

        def f(x):
            return x.dot(x)

        def f_vector(x):
            # Return a vector of size 1.
            return [f(x)]

        def df(x):
            # N.B. We explicitly convert to ndarray since we call this
            # directly.
            x = np.asarray(x)
            return 2 * x

        def g_vector(x):
            # This will be a vector of size 2.
            return [f(x), 3 * f(x)]

        def dg_vector(x):
            # This will be a 2x2 matrix.
            return [df(x), 3 * df(x)]

        def g_matrix(x):
            return [g_vector(x)]

        def dg_matrix(x):
            return [dg_vector(x)]

        numpy_compare.assert_equal(gradient(f, x), df(x))
        numpy_compare.assert_equal(gradient(f_vector, x), df(x))
        numpy_compare.assert_equal(jacobian(f, x), df(x))
        numpy_compare.assert_equal(jacobian(g_vector, x), dg_vector(x))
        numpy_compare.assert_equal(jacobian(g_matrix, x), dg_matrix(x))

    def test_gradient_api_negative(self):
        def f_good(x):
            return x.dot(x)

        x_bad = [[1, 2]]
        with self.assertRaises(AssertionError) as cm:
            gradient(f_good, x_bad)
        self.assertIn("x must be a vector", str(cm.exception))

        def f_bad(_):
            # This will cause an error.
            return [0., 1.]

        with self.assertRaises(AssertionError) as cm:
            gradient(f_bad, x=[10.])
        self.assertIn(
            "must be of a scalar or a vector of size 1", str(cm.exception))

    def test_jacobian_api_negative(self):
        def g_good(x):
            return [x[0], x[1]]

        x_bad = [[1, 2]]
        with self.assertRaises(AssertionError) as cm:
            jacobian(g_good, x_bad)
        self.assertIn("x must be a vector", str(cm.exception))
