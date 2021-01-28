import numpy as np
import unittest

from pydrake.common import ToleranceType
from pydrake.common.test_utilities import numpy_compare
from pydrake.polynomial import Polynomial


class TestPolynomial(unittest.TestCase):
    def test_constructors(self):
        p1 = Polynomial()
        p2 = Polynomial(5)
        p3 = Polynomial([1, 2, 3])

    def test_analysis_methods(self):
        p = Polynomial([1, 2, 3])
        self.assertEqual(p.GetNumberOfCoefficients(), 3)
        self.assertEqual(p.GetDegree(), 2)
        self.assertFalse(p.IsAffine())
        np.testing.assert_equal(p.GetCoefficients(), np.array([1, 2, 3]))
        p_d = p.Derivative(derivative_order=1)
        self.assertEqual(p_d.GetDegree(), 1)
        p_i = p.Integral(integration_constant=0)
        self.assertEqual(p_i.GetDegree(), 3)
        self.assertTrue(
            p.CoefficientsAlmostEqual(p, 1e-14, ToleranceType.kRelative))

    def test_arithmetic(self):
        p = Polynomial([0, 1])
        np.testing.assert_equal((-p).GetCoefficients(), np.array([0, -1]))
        np.testing.assert_equal((p+p).GetCoefficients(), np.array([0, 2]))
        np.testing.assert_equal((p+5).GetCoefficients(), np.array([5, 1]))
        np.testing.assert_equal((5+p).GetCoefficients(), np.array([5, 1]))
        np.testing.assert_equal((p-p).GetCoefficients(), np.array([0]))
        np.testing.assert_equal((p-5).GetCoefficients(), np.array([-5, 1]))
        np.testing.assert_equal((5-p).GetCoefficients(), np.array([5, -1]))
        np.testing.assert_equal((p*p).GetCoefficients(), np.array([0, 0, 1]))
        np.testing.assert_equal((p*5).GetCoefficients(), np.array([0, 5]))
        np.testing.assert_equal((5*p).GetCoefficients(), np.array([0, 5]))
        np.testing.assert_equal((p/5).GetCoefficients(), np.array([0, 0.2]))
        self.assertTrue(p == p)
