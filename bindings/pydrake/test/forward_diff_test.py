import unittest
import numpy as np
from pydrake.forwarddiff import sin, cos, derivative, gradient
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

        def df(x):
            # N.B. We explicitly convert to ndarray since we call this directly.
            x = np.asarray(x)
            return 2 * np.sum(x)

        def g_vector(x):
            return [f(x), 3 * f(x)]

        def dg_vector(x):
            return [df(x), 3 * g(x)]

        numpy_compare.assert_equal(gradient(f, x), df(x))
        numpy_compare.assert_equal(jacobian(g_vector, x), dg_vector(x))
