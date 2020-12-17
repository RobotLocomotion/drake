import numpy as np
import unittest

from pydrake.common import ToleranceType
from pydrake.common.eigen_geometry import AngleAxis, Quaternion
from pydrake.common.test_utilities import numpy_compare
from pydrake.math import BsplineBasis, RotationMatrix
from pydrake.polynomial import Polynomial
from pydrake.trajectories import (
    BsplineTrajectory, PiecewisePolynomial, PiecewiseQuaternionSlerp,
    Trajectory
)


class TestTrajectories(unittest.TestCase):
    def test_trajectory_start_end_time(self):
        # Acceptance check to ensure we have these base methods exposed.
        Trajectory.start_time
        Trajectory.end_time

    def test_bspline_trajectory(self):
        bspline = BsplineTrajectory()
        self.assertIsInstance(bspline, BsplineTrajectory)
        self.assertEqual(BsplineBasis().num_basis_functions(), 0)
        bspline = BsplineTrajectory(
            basis=BsplineBasis(2, [0, 1, 2, 3]),
            control_points=[np.zeros((3, 4)), np.ones((3, 4))])
        self.assertIsInstance(bspline.Clone(), BsplineTrajectory)
        numpy_compare.assert_float_equal(bspline.value(t=1.5),
                                         0.5*np.ones((3, 4)))
        self.assertEqual(bspline.rows(), 3)
        self.assertEqual(bspline.cols(), 4)
        self.assertEqual(bspline.start_time(), 1.)
        self.assertEqual(bspline.end_time(), 2.)
        self.assertEqual(bspline.num_control_points(), 2)
        numpy_compare.assert_float_equal(bspline.control_points()[1],
                                         np.ones((3, 4)))
        numpy_compare.assert_float_equal(bspline.InitialValue(),
                                         np.zeros((3, 4)))
        numpy_compare.assert_float_equal(bspline.FinalValue(), np.ones((3, 4)))
        self.assertIsInstance(bspline.basis(), BsplineBasis)
        bspline.InsertKnots(additional_knots=[1.3, 1.6])
        self.assertEqual(len(bspline.control_points()), 4)
        self.assertIsInstance(
            bspline.CopyBlock(start_row=1, start_col=2,
                              block_rows=2, block_cols=1),
            BsplineTrajectory)
        bspline = BsplineTrajectory(
            basis=BsplineBasis(2, [0, 1, 2, 3]),
            control_points=[np.zeros(3), np.ones(3)])
        self.assertIsInstance(bspline.CopyHead(n=2), BsplineTrajectory)

    def test_piecewise_polynomial_empty_constructor(self):
        pp = PiecewisePolynomial()

    def test_piecewise_polynomial_constant_constructor(self):
        x = np.array([[1.], [4.]])
        pp = PiecewisePolynomial(x)
        self.assertEqual(pp.rows(), 2)
        self.assertEqual(pp.cols(), 1)
        np.testing.assert_equal(x, pp.value(11.))

    def test_piecewise_polynomial_matrix_constructor(self):
        pm1 = np.array([[Polynomial(1), Polynomial(2)]])
        pm2 = np.array([[Polynomial(2), Polynomial(0)]])
        pp = PiecewisePolynomial([pm1, pm2], [0, 1, 2])
        numpy_compare.assert_equal(pp.getPolynomialMatrix(segment_index=0),
                                   pm1)
        pm3 = np.array([[Polynomial(5), Polynomial(10)]])
        pp.setPolynomialMatrixBlock(replacement=pm3, segment_index=1)

    def test_piecewise_polynomial_vector_constructor(self):
        p1 = Polynomial(1)
        p2 = Polynomial(2)
        pp = PiecewisePolynomial([p1, p2], [0, 1, 2])

    def test_zero_order_hold(self):
        x = np.array([[1., 2.], [3., 4.], [5., 6.]]).transpose()
        pp = PiecewisePolynomial.ZeroOrderHold([0., 1., 2.], x)
        pp_d = pp.derivative(derivative_order=1)
        np.testing.assert_equal(np.array([[1.], [2.]]), pp.value(.5))
        np.testing.assert_equal(pp_d.value(.5), np.array([[0.], [0.]]))
        np.testing.assert_equal(pp.EvalDerivative(t=.5, derivative_order=1),
                                np.array([[0.], [0.]]))
        p = pp.getPolynomial(segment_index=0, row=1, col=0)
        np.testing.assert_equal(p.GetCoefficients(), np.array([2]))
        self.assertEqual(pp.getSegmentPolynomialDegree(segment_index=1), 0)
        self.assertEqual(pp.get_number_of_segments(), 2)
        self.assertEqual(pp.start_time(), 0.)
        self.assertEqual(pp.end_time(), 2.)
        self.assertEqual(pp.start_time(segment_index=0), 0.)
        self.assertEqual(pp.end_time(segment_index=0), 1.)
        self.assertEqual(pp.duration(segment_index=0), 1.)
        self.assertTrue(pp.is_time_in_range(t=1.5))
        self.assertEqual(pp.get_segment_index(t=1.5), 1)
        self.assertEqual(pp.get_segment_times(), [0., 1., 2.])

    def test_first_order_hold(self):
        x = np.array([[1., 2.], [3., 4.], [5., 6.]]).transpose()
        pp = PiecewisePolynomial.FirstOrderHold([0., 1., 2.], x)
        np.testing.assert_equal(np.array([[2.], [3.]]), pp.value(.5))

        deriv = pp.MakeDerivative(derivative_order=1)
        np.testing.assert_equal(np.array([[2.], [2.]]), deriv.value(.5))
        pp.AppendFirstOrderSegment(time=3., sample=[-0.4, .57])

    def test_hermite(self):
        t = [0., 1., 2.]
        x = np.array([[0, 1, 1]])
        pp = PiecewisePolynomial.CubicShapePreserving(
            breaks=t, samples=x, zero_end_point_derivatives=False)
        pp.AppendCubicHermiteSegment(time=3., sample=[2], sample_dot=[2])
        pp.RemoveFinalSegment()

    def test_cubic(self):
        t = [0., 1., 2.]
        x = np.diag([4., 5., 6.])
        periodic_end = False
        # Just test the spelling for these.
        pp1 = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            breaks=t, samples=x, periodic_end=periodic_end)
        pp2 = PiecewisePolynomial.CubicHermite(
            breaks=t, samples=x, samples_dot=np.identity(3))
        pp3 = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            breaks=t, samples=x, sample_dot_at_start=[0., 0., 0.],
            sample_dot_at_end=[0., 0., 0.])

    def test_lagrange_interpolating_polynomial(self):
        t = [0., 1., 2.]
        x = np.diag([4., 5., 6.])
        pp = PiecewisePolynomial.LagrangeInterpolatingPolynomial(times=t,
                                                                 samples=x)
        self.assertEqual(pp.get_number_of_segments(), 1)
        np.testing.assert_array_almost_equal(x[:, [1]], pp.value(1.), 1e-12)

    def test_reverse_and_scale_time(self):
        x = np.array([[10.], [20.], [30.]]).transpose()
        pp = PiecewisePolynomial.FirstOrderHold([0.5, 1., 2.], x)
        pp.ReverseTime()
        self.assertEqual(pp.start_time(), -2.0)
        self.assertEqual(pp.end_time(), -0.5)
        pp.ScaleTime(2.0)
        self.assertEqual(pp.start_time(), -4.0)
        self.assertEqual(pp.end_time(), -1.0)

    def test_reshape_and_block(self):
        t = [0., 1., 2., 3.]
        x = np.diag([4., 5., 6., 7.])
        pp = PiecewisePolynomial.FirstOrderHold(t, x)
        self.assertEqual(pp.rows(), 4)
        self.assertEqual(pp.cols(), 1)
        pp.Reshape(rows=2, cols=2)
        self.assertEqual(pp.rows(), 2)
        self.assertEqual(pp.cols(), 2)
        pp2 = pp.Block(start_row=0, start_col=0, block_rows=2, block_cols=1)
        self.assertEqual(pp2.rows(), 2)
        self.assertEqual(pp2.cols(), 1)

    def test_slice_and_shift(self):
        x = np.array([[10.], [20.], [30.]]).transpose()
        pp = PiecewisePolynomial.FirstOrderHold([0., 1., 2.], x)
        pp_sub = pp.slice(start_segment_index=1, num_segments=1)
        self.assertEqual(pp_sub.get_number_of_segments(), 1)
        self.assertEqual(pp_sub.start_time(), 1.)
        self.assertEqual(pp_sub.end_time(), 2.)
        values_sub = np.array(list(map(pp_sub.value, [1., 2.])))
        self.assertTrue((values_sub == [[[20.]], [[30.]]]).all())
        pp_sub.shiftRight(10.)
        self.assertEqual(pp_sub.start_time(), 11.)
        self.assertEqual(pp_sub.end_time(), 12.)

    def test_compare_and_concatenate(self):
        x = np.array([[10.], [20.], [30.]]).transpose()
        pp1 = PiecewisePolynomial.FirstOrderHold([0., 1., 2.], x)
        pp2 = PiecewisePolynomial.FirstOrderHold([2., 3., 4.], x)
        self.assertTrue(pp1.isApprox(
            other=pp1, tol=1e-14, tol_type=ToleranceType.relative))
        pp1.ConcatenateInTime(other=pp2)
        self.assertEqual(pp1.end_time(), 4.)

    def test_vector_values(self):
        pp = PiecewisePolynomial([1, 2, 3])
        v = pp.vector_values([0, 4])
        self.assertEqual(v.shape, (3, 2))

    def test_addition(self):
        pp1 = PiecewisePolynomial([1, 2, 3])
        pp2 = pp1 + pp1
        self.assertTrue((pp2.value(0) == 2 * pp1.value(0)).all())

    def test_quaternion_slerp(self):
        # Test empty constructor.
        pq = PiecewiseQuaternionSlerp()
        self.assertEqual(pq.rows(), 4)
        self.assertEqual(pq.cols(), 1)
        self.assertEqual(pq.get_number_of_segments(), 0)

        t = [0., 1., 2.]
        # Make identity rotations.
        q = Quaternion()
        m = np.identity(3)
        a = AngleAxis()
        R = RotationMatrix()

        # Test quaternion constructor.
        pq = PiecewiseQuaternionSlerp(breaks=t, quaternions=[q, q, q])
        self.assertEqual(pq.get_number_of_segments(), 2)
        np.testing.assert_equal(pq.value(0.5), np.eye(3))

        # Test matrix constructor.
        pq = PiecewiseQuaternionSlerp(breaks=t, rotation_matrices=[m, m, m])
        self.assertEqual(pq.get_number_of_segments(), 2)
        np.testing.assert_equal(pq.value(0.5), np.eye(3))

        # Test axis angle constructor.
        pq = PiecewiseQuaternionSlerp(breaks=t, angle_axes=[a, a, a])
        self.assertEqual(pq.get_number_of_segments(), 2)
        np.testing.assert_equal(pq.value(0.5), np.eye(3))

        # Test rotation matrix constructor.
        pq = PiecewiseQuaternionSlerp(breaks=t, rotation_matrices=[R, R, R])
        self.assertEqual(pq.get_number_of_segments(), 2)
        np.testing.assert_equal(pq.value(0.5), np.eye(3))

        # Test append operations.
        pq.Append(time=3., quaternion=q)
        pq.Append(time=4., rotation_matrix=R)
        pq.Append(time=5., angle_axis=a)
