from __future__ import division, print_function, absolute_import

import unittest
import numpy as np
from pydrake.forwarddiff import sin, cos, derivative


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


if __name__ == '__main__':
    unittest.main()
