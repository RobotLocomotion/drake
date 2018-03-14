from __future__ import print_function

import numpy as np
import unittest

from pydrake.trajectories import (
    PiecewisePolynomial
)


class TestTrajectories(unittest.TestCase):
    def test_piecewise_polynomial_empty_constructor(self):
        pp = PiecewisePolynomial()

    def test_piecewise_polynomial_constant_constructor(self):
        x = np.array([[1.], [4.]])
        pp = PiecewisePolynomial(x)
        self.assertEqual(pp.rows(), 2)
        self.assertEqual(pp.cols(), 1)
        np.testing.assert_equal(x, pp.value(11.))

    def test_zero_order_hold(self):
        x = [[1., 2.], [3., 4.], [5., 6.]]
        pp = PiecewisePolynomial.ZeroOrderHold([0., 1., 2.], x)
        np.testing.assert_equal(np.array([[1.], [2.]]), pp.value(.5))
        self.assertEqual(pp.get_number_of_segments(), 2)
        self.assertEqual(pp.start_time(), 0.)
        self.assertEqual(pp.end_time(), 2.)
        self.assertEqual(pp.start_time(0), 0.)
        self.assertEqual(pp.end_time(0), 1.)
        self.assertEqual(pp.duration(0), 1.)
        self.assertEqual(pp.get_segment_index(1.5), 1)
        self.assertEqual(pp.get_segment_times(), [0., 1., 2.])

    def test_first_order_hold(self):
        x = [[1., 2.], [3., 4.], [5., 6.]]
        pp = PiecewisePolynomial.FirstOrderHold([0., 1., 2.], x)
        np.testing.assert_equal(np.array([[2.], [3.]]), pp.value(.5))

    def test_cubic(self):
        t = [0., 1., 2.]
        x = [[1., 2.], [3., 4.], [5., 6.]]
        # Just test the spelling for these.
        pp2 = PiecewisePolynomial.Cubic(t, x, x)
        pp3 = PiecewisePolynomial.Cubic(t, x, [0., 0.], [0., 0.])
        pp4 = PiecewisePolynomial.Cubic(t, np.identity(3))
