import numpy as np
import unittest

from pydrake.common import ToleranceType
from pydrake.common.test_utilities import numpy_compare
from pydrake.polynomial import Polynomial_


class TestPolynomial(unittest.TestCase):
    @numpy_compare.check_all_types
    def test_constructors(self, T):
        Polynomial = Polynomial_[T]

        p1 = Polynomial()
        p2 = Polynomial(5)
        p3 = Polynomial([1, 2, 3])

    @numpy_compare.check_all_types
    def test_analysis_methods(self, T):
        Polynomial = Polynomial_[T]

        p = Polynomial([1, 2, 3])
        self.assertEqual(p.GetNumberOfCoefficients(), 3)
        self.assertEqual(p.GetDegree(), 2)
        self.assertFalse(p.IsAffine())
        numpy_compare.assert_float_equal(p.GetCoefficients(),
                                         np.array([1., 2., 3.]))
        p_d = p.Derivative(derivative_order=1)
        self.assertEqual(p_d.GetDegree(), 1)
        p_i = p.Integral(integration_constant=0)
        self.assertEqual(p_i.GetDegree(), 3)
        numpy_compare.assert_equal(
            p.CoefficientsAlmostEqual(p, 1e-14, ToleranceType.kRelative), True)

    @numpy_compare.check_all_types
    def test_arithmetic(self, T):
        Polynomial = Polynomial_[T]

        p = Polynomial([0, 1])
        numpy_compare.assert_float_equal((-p).GetCoefficients(),
                                         np.array([0., -1.]))
        numpy_compare.assert_float_equal((p+p).GetCoefficients(),
                                         np.array([0., 2.]))
        numpy_compare.assert_float_equal((p+5).GetCoefficients(),
                                         np.array([5., 1.]))
        numpy_compare.assert_float_equal((5+p).GetCoefficients(),
                                         np.array([5., 1.]))
        numpy_compare.assert_float_equal((p-p).GetCoefficients(),
                                         np.array([0.]))
        numpy_compare.assert_float_equal((p-5).GetCoefficients(),
                                         np.array([-5., 1.]))
        numpy_compare.assert_float_equal((5-p).GetCoefficients(),
                                         np.array([5., -1.]))
        numpy_compare.assert_float_equal((p*p).GetCoefficients(),
                                         np.array([0., 0., 1.]))
        numpy_compare.assert_float_equal((p*5).GetCoefficients(),
                                         np.array([0., 5.]))
        numpy_compare.assert_float_equal((5*p).GetCoefficients(),
                                         np.array([0., 5.]))
        numpy_compare.assert_float_equal((p/5).GetCoefficients(),
                                         np.array([0., 0.2]))
        self.assertTrue(p == p)
