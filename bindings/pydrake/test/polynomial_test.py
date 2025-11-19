import unittest

import numpy as np

from pydrake.common import ToleranceType
from pydrake.common.test_utilities import numpy_compare
from pydrake.polynomial import Polynomial_
from pydrake.symbolic import Expression


class TestPolynomial(unittest.TestCase):
    @numpy_compare.check_all_types
    def test_constructors(self, T):
        Polynomial = Polynomial_[T]

        Polynomial()
        Polynomial(5)
        Polynomial(coefficients=[1, 2, 3])

    @numpy_compare.check_all_types
    def test_analysis_methods(self, T):
        Polynomial = Polynomial_[T]

        p = Polynomial([1, 2, 3])
        self.assertEqual(p.GetNumberOfCoefficients(), 3)
        self.assertEqual(p.GetDegree(), 2)
        self.assertFalse(p.IsAffine())
        numpy_compare.assert_float_equal(
            p.GetCoefficients(), np.array([1.0, 2.0, 3.0])
        )
        p_d = p.Derivative(derivative_order=1)
        self.assertEqual(p_d.GetDegree(), 1)
        p_i = p.Integral(integration_constant=0)
        self.assertEqual(p_i.GetDegree(), 3)
        numpy_compare.assert_equal(
            p.CoefficientsAlmostEqual(p, 1e-14, ToleranceType.kRelative), True
        )

        x = 1.3
        y = p.EvaluateUnivariate(x=x, derivative_order=0)
        if T == Expression:
            self.assertTrue(y.EqualTo(1 + 2 * x + 3 * x**2))
        else:
            self.assertAlmostEqual(y, 1 + 2 * x + 3 * x**2)

    @numpy_compare.check_all_types
    def test_arithmetic(self, T):
        Polynomial = Polynomial_[T]

        p = Polynomial([0, 1])
        numpy_compare.assert_float_equal(
            (-p).GetCoefficients(), np.array([0.0, -1.0])
        )
        numpy_compare.assert_float_equal(
            (p + p).GetCoefficients(), np.array([0.0, 2.0])
        )
        numpy_compare.assert_float_equal(
            (p + 5).GetCoefficients(), np.array([5.0, 1.0])
        )
        numpy_compare.assert_float_equal(
            (5 + p).GetCoefficients(), np.array([5.0, 1.0])
        )
        numpy_compare.assert_float_equal(
            (p - p).GetCoefficients(), np.array([0.0])
        )
        numpy_compare.assert_float_equal(
            (p - 5).GetCoefficients(), np.array([-5.0, 1.0])
        )
        numpy_compare.assert_float_equal(
            (5 - p).GetCoefficients(), np.array([5.0, -1.0])
        )
        numpy_compare.assert_float_equal(
            (p * p).GetCoefficients(), np.array([0.0, 0.0, 1.0])
        )
        numpy_compare.assert_float_equal(
            (p * 5).GetCoefficients(), np.array([0.0, 5.0])
        )
        numpy_compare.assert_float_equal(
            (5 * p).GetCoefficients(), np.array([0.0, 5.0])
        )
        numpy_compare.assert_float_equal(
            (p / 5).GetCoefficients(), np.array([0.0, 0.2])
        )
        self.assertTrue(p == p)
