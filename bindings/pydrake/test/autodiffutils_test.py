from __future__ import print_function
# N.B. We are purposely not importing `division` to test nominal Python2
# behavior.

import pydrake.autodiffutils as mut
from pydrake.autodiffutils import AutoDiffXd

import unittest
import numpy as np
import pydrake.math as drake_math

# Use convenience abbreviation.
AD = AutoDiffXd


class TestAutoDiffXd(unittest.TestCase):
    def test_api(self):
        a = AD(1, [1., 0])
        self.assertEquals(a.value(), 1.)
        self.assertTrue((a.derivatives() == [1., 0]).all())
        self.assertEquals(str(a), "AD{1.0, nderiv=2}")
        self.assertEquals(repr(a), "<AutoDiffXd 1.0 nderiv=2>")

    def _compare_scalar(self, actual, expected):
        self.assertAlmostEquals(actual.value(), expected.value())
        self.assertTrue(
            (actual.derivatives() == expected.derivatives()).all(),
            (actual.derivatives(), expected.derivatives()))

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
        # Test autodiff overloads.
        # See `math_overloads_test` for more comprehensive checks.
        c = AD(0, [1., 0])
        d = AD(1, [0, 1.])
        self._compare_scalar(drake_math.sin(c), AD(0, [1, 0]))
        self._compare_scalar(drake_math.cos(c), AD(1, [0, 0]))
        self._compare_scalar(drake_math.tan(c), AD(0, [1, 0]))
        self._compare_scalar(drake_math.asin(c), AD(0, [1, 0]))
        self._compare_scalar(drake_math.acos(c), AD(np.pi / 2, [-1, 0]))
        self._compare_scalar(drake_math.atan2(c, d), AD(0, [1, 0]))
        self._compare_scalar(drake_math.sinh(c), AD(0, [1, 0]))
        self._compare_scalar(drake_math.cosh(c), AD(1, [0, 0]))
        self._compare_scalar(drake_math.tanh(c), AD(0, [1, 0]))
