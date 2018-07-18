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
        x = np.array([[1., 2.], [3., 4.], [5., 6.]]).transpose()
        pp = PiecewisePolynomial.ZeroOrderHold([0., 1., 2.], x)
        pp_d = pp.derivative(1)
        np.testing.assert_equal(np.array([[1.], [2.]]), pp.value(.5))
        np.testing.assert_equal(pp_d.value(.5), np.array([[0.], [0.]]))
        self.assertEqual(pp.get_number_of_segments(), 2)
        self.assertEqual(pp.start_time(), 0.)
        self.assertEqual(pp.end_time(), 2.)
        self.assertEqual(pp.start_time(0), 0.)
        self.assertEqual(pp.end_time(0), 1.)
        self.assertEqual(pp.duration(0), 1.)
        self.assertEqual(pp.get_segment_index(1.5), 1)
        self.assertEqual(pp.get_segment_times(), [0., 1., 2.])

    def test_first_order_hold(self):
        x = np.array([[1., 2.], [3., 4.], [5., 6.]]).transpose()
        pp = PiecewisePolynomial.FirstOrderHold([0., 1., 2.], x)
        np.testing.assert_equal(np.array([[2.], [3.]]), pp.value(.5))

    def test_cubic(self):
        t = [0., 1., 2.]
        x = np.diag((4., 5., 6.))
        periodic_end = False
        # Just test the spelling for these.
        pp1 = PiecewisePolynomial.Cubic(t, x, periodic_end)
        pp2 = PiecewisePolynomial.Cubic(t, x, np.identity(3))
        pp3 = PiecewisePolynomial.Cubic(t, x, [0., 0., 0.], [0., 0., 0.])

    def test_slice_and_shift(self):
        x = np.array([[10.], [20.], [30.]]).transpose()
        pp = PiecewisePolynomial.FirstOrderHold([0., 1., 2.], x)
        pp_sub = pp.slice(start_segment_index=1, num_segments=1)
        self.assertEqual(pp_sub.get_number_of_segments(), 1)
        self.assertEqual(pp_sub.start_time(), 1.)
        self.assertEqual(pp_sub.end_time(), 2.)
        values_sub = np.array(map(pp_sub.value, [1., 2.]))
        self.assertTrue((values_sub == [[[20.]], [[30.]]]).all())
        pp_sub.shiftRight(10.)
        self.assertEqual(pp_sub.start_time(), 11.)
        self.assertEqual(pp_sub.end_time(), 12.)
