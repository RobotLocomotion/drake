from __future__ import print_function
# N.B. We are purposely not importing `division` to test nominal Python2
# behavior.

import pydrake.autodiffutils as mut
from pydrake.autodiffutils import AutoDiffXd

import unittest
import numpy as np
from pydrake.math import sin, cos

# Use convenience abbreviation.
AD = AutoDiffXd


class TestAutoDiffXd(unittest.TestCase):
    def _compare_scalar(self, actual, expected):
        self.assertAlmostEquals(actual.value(), expected.value())
        self.assertTrue((actual.derivatives() == expected.derivatives()).all())

    def test_scalar_math(self):
        a = AD(1, [1., 0])
        self._compare_scalar(a, a)
        b = AD(2, [0, 1.])
        self._compare_scalar(a + b, AD(3, [1, 1]))
        self._compare_scalar(a + 1, AD(2, [1, 0]))
        self._compare_scalar(1 + a, AD(2, [1, 0]))
        self._compare_scalar(a - b, AD(-1, [1, -1]))
        self._compare_scalar(a - 1, AD(0, [1, 0]))
        self._compare_scalar(1 - a, AD(0, [-1, 0]))
        self._compare_scalar(a * b, AD(2, [2, 1]))
        self._compare_scalar(a * 2, AD(2, [2, 0]))
        self._compare_scalar(2 * a, AD(2, [2, 0]))
        self._compare_scalar(a / b, AD(1./2, [1./2, -1./4]))
        self._compare_scalar(a / 2, AD(0.5, [0.5, 0]))
        self._compare_scalar(2 / a, AD(2, [-2, 0]))
        self._compare_scalar(a**2, AD(1, [2., 0]))
        # Test autodiff and float overloads.
        c = AD(0, [1., 0])
        fc = 0
        self.assertEquals(sin(fc), 0)
        self._compare_scalar(sin(c), AD(0, [1, 0]))
        self.assertEquals(cos(fc), 1)
        self._compare_scalar(cos(c), AD(1, [0, 0]))

    def test_backwards_compatibility(self):
        c = AD(0, [1., 0])
        self._compare_scalar(c.sin(), sin(c))
        self._compare_scalar(c.cos(), cos(c))
