import copy
import numpy as np
import pickle
from textwrap import dedent
import unittest
import scipy.sparse

from pydrake.common import ToleranceType
from pydrake.common.eigen_geometry import AngleAxis_, Quaternion_
from pydrake.common.test_utilities import numpy_compare
from pydrake.common.test_utilities.pickle_compare import assert_pickle
from pydrake.common.value import AbstractValue
from pydrake.common.yaml import yaml_load_typed
from pydrake.math import BsplineBasis_, RigidTransform_, RotationMatrix_
from pydrake.polynomial import Polynomial_
from pydrake.trajectories import (
    BezierCurve_,
    BsplineTrajectory_,
    CompositeTrajectory_,
    PathParameterizedTrajectory_,
    PiecewisePolynomial_,
    PiecewisePose_,
    PiecewiseQuaternionSlerp_,
    StackedTrajectory_,
    Trajectory,
    Trajectory_
)
from pydrake.symbolic import Variable, Expression


# Custom trajectory class used to test Trajectory subclassing in python.
class CustomTrajectory(Trajectory):
    def __init__(self):
        Trajectory.__init__(self)

    def rows(self):
        return 1

    def cols(self):
        return 2

    def start_time(self):
        return 3.0

    def end_time(self):
        return 4.0

    def value(self, t):
        return np.array([[t + 1.0, t + 2.0]])

    def do_has_derivative(self):
        return True

    def DoEvalDerivative(self, t, derivative_order):
        if derivative_order >= 2:
            return np.zeros((1, 2))
        elif derivative_order == 1:
            return np.ones((1, 2))
        elif derivative_order == 0:
            return self.value(t)


class TestTrajectories(unittest.TestCase):
    @numpy_compare.check_all_types
    def test_trajectory_start_end_time(self, T):
        # Acceptance check to ensure we have these base methods exposed.
        Trajectory_[T].start_time
        Trajectory_[T].end_time

    def test_custom_trajectory(self):
        trajectory = CustomTrajectory()
        self.assertEqual(trajectory.rows(), 1)
        self.assertEqual(trajectory.cols(), 2)
        self.assertEqual(trajectory.start_time(), 3.0)
        self.assertEqual(trajectory.end_time(), 4.0)
        self.assertTrue(trajectory.has_derivative())
        numpy_compare.assert_float_equal(trajectory.value(t=1.5),
                                         np.array([[2.5, 3.5]]))
        numpy_compare.assert_float_equal(
            trajectory.EvalDerivative(t=2.3, derivative_order=1),
            np.ones((1, 2)))
        numpy_compare.assert_float_equal(
            trajectory.EvalDerivative(t=2.3, derivative_order=2),
            np.zeros((1, 2)))

    @numpy_compare.check_all_types
    def test_bezier_curve(self, T):
        curve = BezierCurve_[T]()
        self.assertEqual(curve.rows(), 0)
        self.assertEqual(curve.cols(), 1)

        points = np.mat("4.0, 5.0; 6.0, 7.0")
        curve = BezierCurve_[T](start_time=1,
                                end_time=2,
                                control_points=points)
        numpy_compare.assert_float_equal(curve.start_time(), 1.0)
        numpy_compare.assert_float_equal(curve.end_time(), 2.0)
        self.assertEqual(curve.rows(), 2)
        self.assertEqual(curve.cols(), 1)
        self.assertEqual(curve.order(), 1)

        b = curve.BernsteinBasis(i=0, time=1.5, order=1)
        self.assertIsInstance(b, T)
        numpy_compare.assert_float_equal(curve.control_points(), points)

        M = curve.AsLinearInControlPoints(derivative_order=1)
        self.assertEqual(M.shape, (2, 1))
        self.assertIsInstance(M, scipy.sparse.csc_matrix)

        curve_expression = curve.GetExpression(time=Variable("t"))
        self.assertEqual(curve_expression.shape, (2,))
        for expr in curve_expression:
            self.assertTrue(isinstance(expr, Expression))

        curve.ElevateOrder()
        self.assertEqual(curve.order(), 2)

    @numpy_compare.check_all_types
    def test_bspline_trajectory(self, T):
        BsplineBasis = BsplineBasis_[T]
        BsplineTrajectory = BsplineTrajectory_[T]

        # Call the default constructor.
        bspline = BsplineTrajectory()
        self.assertIsInstance(bspline, BsplineTrajectory)
        self.assertEqual(BsplineBasis().num_basis_functions(), 0)
        # Call the vector<vector<T>> constructor.
        bspline = BsplineTrajectory(basis=BsplineBasis(2, [0, 1, 2, 3]),
                                    control_points=np.zeros((4, 2)))
        self.assertEqual(bspline.rows(), 4)
        self.assertEqual(bspline.cols(), 1)
        # Call the vector<MatrixX<T>> constructor.
        bspline = BsplineTrajectory(
            basis=BsplineBasis(2, [0, 1, 2, 3]),
            control_points=[np.zeros((3, 4)), np.ones((3, 4))])
        self.assertIsInstance(bspline.Clone(), BsplineTrajectory)
        numpy_compare.assert_float_equal(bspline.value(t=1.5),
                                         0.5*np.ones((3, 4)))
        self.assertEqual(bspline.rows(), 3)
        self.assertEqual(bspline.cols(), 4)
        numpy_compare.assert_float_equal(bspline.start_time(), 1.)
        numpy_compare.assert_float_equal(bspline.end_time(), 2.)
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
        bspline = BsplineTrajectory(basis=BsplineBasis(2, [0, 1, 2, 3]),
                                    control_points=np.array([[0, 1], [0, 1],
                                                             [0, 1]]))
        self.assertIsInstance(bspline.CopyHead(n=2), BsplineTrajectory)
        # Ensure we can copy.
        self.assertEqual(copy.copy(bspline).rows(), 3)
        self.assertEqual(copy.deepcopy(bspline).rows(), 3)
        assert_pickle(self, bspline,
                      lambda traj: np.array(traj.control_points()), T=T)

    def test_legacy_unpickle(self):
        """Checks that data pickled as BsplineTrajectory_[float] in Drake
        v1.12.0 can be unpickled as BsplineTrajectory_𝓣float𝓤 in newer
        versions of Drake.
        """
        legacy_data = b"\x80\x04\x95q\x01\x00\x00\x00\x00\x00\x00\x8c\x14pydrake.trajectories\x94\x8c\x19BsplineTrajectory_[float]\x94\x93\x94)\x81\x94\x8c\x0cpydrake.math\x94\x8c\x14BsplineBasis_[float]\x94\x93\x94)\x81\x94K\x02]\x94(G\x00\x00\x00\x00\x00\x00\x00\x00G?\xf0\x00\x00\x00\x00\x00\x00G@\x00\x00\x00\x00\x00\x00\x00G@\x08\x00\x00\x00\x00\x00\x00e\x86\x94b]\x94(\x8c\x15numpy.core.multiarray\x94\x8c\x0c_reconstruct\x94\x93\x94\x8c\x05numpy\x94\x8c\x07ndarray\x94\x93\x94K\x00\x85\x94C\x01b\x94\x87\x94R\x94(K\x01K\x03K\x01\x86\x94h\x0e\x8c\x05dtype\x94\x93\x94\x8c\x02f8\x94\x89\x88\x87\x94R\x94(K\x03\x8c\x01<\x94NNNJ\xff\xff\xff\xffJ\xff\xff\xff\xffK\x00t\x94b\x89C\x18\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x94t\x94bh\rh\x10K\x00\x85\x94h\x12\x87\x94R\x94(K\x01K\x03K\x01\x86\x94h\x1a\x89C\x18\x00\x00\x00\x00\x00\x00\xf0?\x00\x00\x00\x00\x00\x00\xf0?\x00\x00\x00\x00\x00\x00\xf0?\x94t\x94be\x86\x94b."  # noqa
        obj = pickle.loads(legacy_data)
        self.assertIsInstance(obj, BsplineTrajectory_[float])
        self.assertEqual(obj.rows(), 3)
        self.assertEqual(obj.cols(), 1)

    @numpy_compare.check_all_types
    def test_path_parameterized_trajectory(self, T):
        PathParameterizedTrajectory = PathParameterizedTrajectory_[T]
        PiecewisePolynomial = PiecewisePolynomial_[T]

        s = np.array([[1., 3., 5.]])
        x = np.array([[1., 2.], [3., 4.], [5., 6.]]).transpose()
        param = PiecewisePolynomial.FirstOrderHold([0., 1., 2.], s)
        path = PiecewisePolynomial.ZeroOrderHold(s[0], x)
        trajectory = PathParameterizedTrajectory(path, param)
        self.assertIsInstance(trajectory.Clone(), PathParameterizedTrajectory)
        numpy_compare.assert_float_equal(trajectory.value(t=1.5), x[:, 1:2])
        self.assertEqual(trajectory.rows(), 2)
        self.assertEqual(trajectory.cols(), 1)
        numpy_compare.assert_float_equal(trajectory.start_time(), 0.)
        numpy_compare.assert_float_equal(trajectory.end_time(), 2.)
        self.assertIsInstance(trajectory.path(), PiecewisePolynomial)
        self.assertIsInstance(trajectory.time_scaling(),
                              PiecewisePolynomial)

    @numpy_compare.check_all_types
    def test_piecewise_polynomial_empty_constructor(self, T):
        PiecewisePolynomial = PiecewisePolynomial_[T]

        pp = PiecewisePolynomial()
        # Ensure we can copy.
        copy.copy(pp)
        copy.deepcopy(pp)
        self.assertEqual(copy.copy(pp).get_number_of_segments(), 0)
        self.assertEqual(copy.deepcopy(pp).get_number_of_segments(), 0)

    @numpy_compare.check_all_types
    def test_piecewise_polynomial_constant_constructor(self, T):
        PiecewisePolynomial = PiecewisePolynomial_[T]

        x = np.array([[1.], [4.]])
        pp = PiecewisePolynomial(x)
        self.assertEqual(pp.rows(), 2)
        self.assertEqual(pp.cols(), 1)
        numpy_compare.assert_float_equal(pp.value(11.), x)
        # Ensure we can copy.
        self.assertEqual(copy.copy(pp).rows(), 2)
        self.assertEqual(copy.deepcopy(pp).rows(), 2)

    @numpy_compare.check_all_types
    def test_piecewise_polynomial_matrix_constructor(self, T):
        Polynomial = Polynomial_[T]
        PiecewisePolynomial = PiecewisePolynomial_[T]

        pm1 = np.array([[Polynomial(1), Polynomial(2)]])
        pm2 = np.array([[Polynomial(2), Polynomial(0)]])
        pp = PiecewisePolynomial([pm1, pm2], [0, 1, 2])
        numpy_compare.assert_equal(pp.getPolynomialMatrix(segment_index=0),
                                   pm1)
        pm3 = np.array([[Polynomial(5), Polynomial(10)]])
        pp.setPolynomialMatrixBlock(replacement=pm3, segment_index=1)
        # Ensure we can copy.
        self.assertEqual(copy.copy(pp).cols(), 2)
        self.assertEqual(copy.deepcopy(pp).cols(), 2)

    @numpy_compare.check_all_types
    def test_piecewise_polynomial_vector_constructor(self, T):
        Polynomial = Polynomial_[T]
        PiecewisePolynomial = PiecewisePolynomial_[T]

        p1 = Polynomial(1)
        p2 = Polynomial(2)
        pp = PiecewisePolynomial([p1, p2], [0, 1, 2])
        # Ensure we can copy.
        self.assertEqual(copy.copy(pp).rows(), 1)
        self.assertEqual(copy.deepcopy(pp).rows(), 1)

    def test_piecewise_polynomial_serialize(self):
        PiecewisePolynomial = PiecewisePolynomial_[float]
        breaks = [0, 0.5, 1]
        sample0 = np.array([[1, 1, 2], [2, 0, 3]])
        sample1 = np.array([[3, 4, 5], [6, 7, 8]])
        sample2 = np.zeros((2, 3))
        expected = PiecewisePolynomial.ZeroOrderHold(
            breaks=breaks,
            samples=[sample0, sample1, sample2])
        data = dedent("""
        breaks: [0.0, 0.5, 1.0]
        polynomials:
          -
            - [[1], [1], [2]]
            - [[2], [0], [3]]
          -
            - [[3], [4], [5]]
            - [[6], [7], [8]]
        """)
        dut = yaml_load_typed(schema=PiecewisePolynomial, data=data)
        self.assertEqual(dut.get_number_of_segments(), 2)
        self.assertEqual(dut.rows(), 2)
        self.assertEqual(dut.cols(), 3)
        self.assertTrue(dut.isApprox(expected, tol=0))

    def test_piecewise_polynomial_serialize_empty(self):
        data = dedent("""
        breaks: []
        polynomials: []
        """)
        dut = yaml_load_typed(schema=PiecewisePolynomial_[float], data=data)
        self.assertEqual(dut.get_number_of_segments(), 0)

    @numpy_compare.check_all_types
    def test_zero_order_hold_vector(self, T):
        PiecewisePolynomial = PiecewisePolynomial_[T]

        x = np.array([[1., 2.], [3., 4.], [5., 6.]]).transpose()
        pp = PiecewisePolynomial.ZeroOrderHold([0., 1., 2.], x)
        pp_d = pp.derivative(derivative_order=1)
        numpy_compare.assert_float_equal(pp.value(.5), np.array([[1.], [2.]]))
        numpy_compare.assert_float_equal(
            pp_d.value(.5), np.array([[0.], [0.]]))
        numpy_compare.assert_float_equal(
            pp.EvalDerivative(t=.5, derivative_order=1),
            np.array([[0.], [0.]]))
        p = pp.getPolynomial(segment_index=0, row=1, col=0)
        numpy_compare.assert_float_equal(p.GetCoefficients(), np.array([2.]))
        self.assertEqual(pp.getSegmentPolynomialDegree(segment_index=1), 0)
        self.assertEqual(pp.get_number_of_segments(), 2)
        numpy_compare.assert_float_equal(pp.start_time(), 0.)
        numpy_compare.assert_float_equal(pp.end_time(), 2.)
        self.assertEqual(pp.rows(), 2)
        self.assertEqual(pp.cols(), 1)
        numpy_compare.assert_float_equal(pp.start_time(segment_index=0), 0.)
        numpy_compare.assert_float_equal(pp.end_time(segment_index=0), 1.)
        numpy_compare.assert_float_equal(pp.duration(segment_index=0), 1.)
        numpy_compare.assert_equal(pp.is_time_in_range(t=1.5), True)
        self.assertEqual(pp.get_segment_index(t=1.5), 1)
        numpy_compare.assert_float_equal(pp.get_segment_times(), [0., 1., 2.])

    @numpy_compare.check_all_types
    def test_first_order_hold_vector(self, T):
        PiecewisePolynomial = PiecewisePolynomial_[T]

        x = np.array([[1., 2.], [3., 4.], [5., 6.]]).transpose()
        pp = PiecewisePolynomial.FirstOrderHold([0., 1., 2.], x)
        numpy_compare.assert_float_equal(pp.value(.5), np.array([[2.], [3.]]))

        deriv = pp.MakeDerivative(derivative_order=1)
        numpy_compare.assert_float_equal(deriv.value(.5),
                                         np.array([[2.], [2.]]))
        pp.AppendFirstOrderSegment(time=3., sample=[-0.4, .57])

    @numpy_compare.check_all_types
    def test_hermite_vector(self, T):
        PiecewisePolynomial = PiecewisePolynomial_[T]

        t = [0., 1., 2.]
        x = np.array([[0, 1, 1]])
        pp = PiecewisePolynomial.CubicShapePreserving(
            breaks=t, samples=x, zero_end_point_derivatives=False)
        pp.AppendCubicHermiteSegment(time=3., sample=[2], sample_dot=[2])
        pp.RemoveFinalSegment()

    @numpy_compare.check_all_types
    def test_cubic_vector(self, T):
        PiecewisePolynomial = PiecewisePolynomial_[T]

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

    @numpy_compare.check_all_types
    def test_lagrange_interpolating_polynomial_vector(self, T):
        PiecewisePolynomial = PiecewisePolynomial_[T]

        t = [0., 1., 2.]
        x = np.diag([4., 5., 6.])
        pp = PiecewisePolynomial.LagrangeInterpolatingPolynomial(times=t,
                                                                 samples=x)
        self.assertEqual(pp.get_number_of_segments(), 1)
        numpy_compare.assert_float_allclose(pp.value(1.), x[:, [1]], 1e-12)

    @numpy_compare.check_all_types
    def test_matrix_trajectories(self, T):
        PiecewisePolynomial = PiecewisePolynomial_[T]

        A0 = np.array([[1., 2., 3.], [4., 5., 6.]])
        A1 = np.array([[7., 8., 9.], [10., 11., 12.]])
        A2 = np.array([[13., 14., 15.], [16., 17., 18.]])
        t = [0., 1., 2.]
        pp = dict()
        pp["zoh"] = PiecewisePolynomial.ZeroOrderHold(
            breaks=t, samples=[A0, A1, A2])
        pp["foh"] = PiecewisePolynomial.FirstOrderHold(
            breaks=t, samples=[A0, A1, A2])
        pp["hermite"] = PiecewisePolynomial.CubicShapePreserving(
            breaks=t, samples=[A0, A1, A2], zero_end_point_derivatives=False)
        pp["c1"] = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            breaks=t, samples=[A0, A1, A2], periodic_end=False)
        pp["c2"] = PiecewisePolynomial.CubicHermite(
            breaks=t, samples=[A0, A1, A2], samples_dot=[0*A0, 0*A1, 0*A2])
        pp["c3"] = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            breaks=t, samples=[A0, A1, A2], sample_dot_at_start=0*A0,
            sample_dot_at_end=0*A0)
        pp["lagrange"] = PiecewisePolynomial.LagrangeInterpolatingPolynomial(
            times=t, samples=[A0, A1, A2])
        for name, traj in pp.items():
            if name == "lagrange":
                self.assertEqual(traj.get_number_of_segments(), 1)
            else:
                self.assertEqual(traj.get_number_of_segments(), 2)
            numpy_compare.assert_float_equal(traj.start_time(), 0.)
            numpy_compare.assert_float_equal(traj.end_time(), 2.)
            self.assertEqual(traj.rows(), 2)
            self.assertEqual(traj.cols(), 3)
        # Check the values for the easy cases:
        numpy_compare.assert_float_equal(pp["zoh"].value(.5), A0)
        numpy_compare.assert_float_allclose(
            pp["foh"].value(.5), 0.5*A0 + 0.5*A1, 1e-15)

    @numpy_compare.check_all_types
    def test_reverse_and_scale_time(self, T):
        PiecewisePolynomial = PiecewisePolynomial_[T]

        x = np.array([[10.], [20.], [30.]]).transpose()
        pp = PiecewisePolynomial.FirstOrderHold([0.5, 1., 2.], x)
        pp.ReverseTime()
        numpy_compare.assert_float_equal(pp.start_time(), -2.0)
        numpy_compare.assert_float_equal(pp.end_time(), -0.5)
        pp.ScaleTime(2.0)
        numpy_compare.assert_float_equal(pp.start_time(), -4.0)
        numpy_compare.assert_float_equal(pp.end_time(), -1.0)

    @numpy_compare.check_all_types
    def test_reshape_block_and_transpose(self, T):
        PiecewisePolynomial = PiecewisePolynomial_[T]

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
        pp3 = pp2.Transpose()
        self.assertEqual(pp3.rows(), 1)
        self.assertEqual(pp3.cols(), 2)

    @numpy_compare.check_all_types
    def test_slice_and_shift(self, T):
        PiecewisePolynomial = PiecewisePolynomial_[T]

        x = np.array([[10.], [20.], [30.]]).transpose()
        pp = PiecewisePolynomial.FirstOrderHold([0., 1., 2.], x)
        pp_sub = pp.slice(start_segment_index=1, num_segments=1)
        self.assertEqual(pp_sub.get_number_of_segments(), 1)
        numpy_compare.assert_float_equal(pp_sub.start_time(), 1.)
        numpy_compare.assert_float_equal(pp_sub.end_time(), 2.)
        values_sub = np.array(list(map(pp_sub.value, [1., 2.])))
        numpy_compare.assert_float_equal(values_sub, [[[20.]], [[30.]]])
        pp_sub.shiftRight(10.)
        numpy_compare.assert_float_equal(pp_sub.start_time(), 11.)
        numpy_compare.assert_float_equal(pp_sub.end_time(), 12.)

    @numpy_compare.check_all_types
    def test_compare_and_concatenate(self, T):
        PiecewisePolynomial = PiecewisePolynomial_[T]

        x = np.array([[10.], [20.], [30.]]).transpose()
        pp1 = PiecewisePolynomial.FirstOrderHold([0., 1., 2.], x)
        pp2 = PiecewisePolynomial.FirstOrderHold([2., 3., 4.], x)
        self.assertTrue(pp1.isApprox(
            other=pp1, tol=1e-14, tol_type=ToleranceType.kRelative))
        pp1.ConcatenateInTime(other=pp2)
        numpy_compare.assert_float_equal(pp1.end_time(), 4.)

    @numpy_compare.check_all_types
    def test_vector_values(self, T):
        PiecewisePolynomial = PiecewisePolynomial_[T]

        pp = PiecewisePolynomial([1, 2, 3])
        v = pp.vector_values([0, 4])
        self.assertEqual(v.shape, (3, 2))

    @numpy_compare.check_all_types
    def test_addition(self, T):
        PiecewisePolynomial = PiecewisePolynomial_[T]

        pp1 = PiecewisePolynomial([1, 2, 3])
        pp2 = pp1 + pp1
        numpy_compare.assert_equal(pp2.value(0), 2 * pp1.value(0))

    @numpy_compare.check_all_types
    def test_composite_trajectory(self, T):
        CompositeTrajectory = CompositeTrajectory_[T]
        PiecewisePolynomial = PiecewisePolynomial_[T]

        x = np.array([[10.0, 20.0, 30.0]])
        pp1 = PiecewisePolynomial.FirstOrderHold([0.0, 1.0, 2.0], x)
        pp2 = PiecewisePolynomial.FirstOrderHold([2.0, 3.0, 4.0], x)
        traj = CompositeTrajectory(segments=[pp1, pp2])
        self.assertEqual(traj.get_number_of_segments(), 2)
        self.assertEqual(traj.rows(), 1)
        self.assertEqual(traj.cols(), 1)
        numpy_compare.assert_float_equal(traj.start_time(), 0.0)
        numpy_compare.assert_float_equal(traj.end_time(), 4.0)
        self.assertIsInstance(traj.segment(segment_index=0),
                              PiecewisePolynomial)
        self.assertIsInstance(traj.segment(segment_index=1),
                              PiecewisePolynomial)

    @numpy_compare.check_all_types
    def test_quaternion_slerp(self, T):
        AngleAxis = AngleAxis_[T]
        PiecewiseQuaternionSlerp = PiecewiseQuaternionSlerp_[T]
        Quaternion = Quaternion_[T]
        RotationMatrix = RotationMatrix_[T]

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
        numpy_compare.assert_float_equal(pq.value(0.5),
                                         [[1.], [0.], [0.], [0.]])

        # Test matrix constructor.
        pq = PiecewiseQuaternionSlerp(breaks=t, rotation_matrices=[m, m, m])
        self.assertEqual(pq.get_number_of_segments(), 2)
        numpy_compare.assert_float_equal(pq.value(0.5),
                                         [[1.], [0.], [0.], [0.]])

        # Test axis angle constructor.
        pq = PiecewiseQuaternionSlerp(breaks=t, angle_axes=[a, a, a])
        self.assertEqual(pq.get_number_of_segments(), 2)
        numpy_compare.assert_float_equal(pq.value(0.5),
                                         [[1.], [0.], [0.], [0.]])

        # Test rotation matrix constructor.
        pq = PiecewiseQuaternionSlerp(breaks=t, rotation_matrices=[R, R, R])
        self.assertEqual(pq.get_number_of_segments(), 2)
        numpy_compare.assert_float_equal(pq.value(0.5),
                                         [[1.], [0.], [0.], [0.]])

        # Test append operations.
        pq.Append(time=3., quaternion=q)
        pq.Append(time=4., rotation_matrix=R)
        pq.Append(time=5., angle_axis=a)

        # Test getters.
        pq = PiecewiseQuaternionSlerp(
            breaks=[0, 1], angle_axes=[a, AngleAxis(np.pi/2, [0, 0, 1])]
        )
        numpy_compare.assert_float_equal(
            pq.orientation(time=0).wxyz(), np.array([1., 0., 0., 0.])
        )
        numpy_compare.assert_float_allclose(
            pq.angular_velocity(time=0), np.array([0, 0, np.pi/2]),
            atol=1e-15, rtol=0,
        )
        numpy_compare.assert_float_equal(
            pq.angular_acceleration(time=0), np.zeros(3)
        )

        numpy_compare.assert_float_allclose(
            pq.orientation(time=1).wxyz(),
            np.array([np.cos(np.pi/4), 0, 0, np.sin(np.pi/4)]),
            atol=1e-15, rtol=0,
        )
        numpy_compare.assert_float_allclose(
            pq.angular_velocity(time=1), np.array([0, 0, np.pi/2]),
            atol=1e-15, rtol=0,
        )
        numpy_compare.assert_float_equal(
            pq.angular_acceleration(time=1), np.zeros(3)
        )
        # Ensure we can copy.
        numpy_compare.assert_float_allclose(
            copy.copy(pq).orientation(time=1).wxyz(),
            np.array([np.cos(np.pi/4), 0, 0, np.sin(np.pi/4)]),
            atol=1e-15, rtol=0,
        )
        numpy_compare.assert_float_allclose(
            copy.deepcopy(pq).orientation(time=1).wxyz(),
            np.array([np.cos(np.pi/4), 0, 0, np.sin(np.pi/4)]),
            atol=1e-15, rtol=0,
        )

    @numpy_compare.check_all_types
    def test_piecewise_pose(self, T):
        PiecewisePolynomial = PiecewisePolynomial_[T]
        PiecewisePose = PiecewisePose_[T]
        PiecewiseQuaternionSlerp = PiecewiseQuaternionSlerp_[T]
        Quaternion = Quaternion_[T]
        RigidTransform = RigidTransform_[T]

        # Test empty constructor.
        ppose = PiecewisePose()
        self.assertEqual(ppose.rows(), 4)
        self.assertEqual(ppose.cols(), 4)
        self.assertEqual(ppose.get_number_of_segments(), 0)

        t = [0., 1., 2.]
        q = Quaternion()
        pp = PiecewisePolynomial.FirstOrderHold(t, np.zeros((3, 3)))
        pq = PiecewiseQuaternionSlerp(t, [q, q, q])
        ppose = PiecewisePose(position_trajectory=pp,
                              orientation_trajectory=pq)
        self.assertEqual(ppose.get_number_of_segments(), 2)

        numpy_compare.assert_float_equal(ppose.GetPose(
            time=0).GetAsMatrix4(), np.eye(4))
        numpy_compare.assert_float_equal(
            ppose.GetVelocity(time=0), np.zeros((6,)))
        numpy_compare.assert_float_equal(
            ppose.GetAcceleration(time=0), np.zeros((6,)))
        self.assertTrue(ppose.IsApprox(other=ppose, tol=0.0))
        self.assertIsInstance(
            ppose.get_position_trajectory(), PiecewisePolynomial)
        self.assertIsInstance(
            ppose.get_orientation_trajectory(), PiecewiseQuaternionSlerp)

        X = RigidTransform()
        ppose = PiecewisePose.MakeLinear(times=t, poses=[X, X, X])
        self.assertEqual(ppose.get_number_of_segments(), 2)

        ppose = PiecewisePose.MakeCubicLinearWithEndLinearVelocity(
            times=t, poses=[X, X, X], start_vel=np.zeros((3, 1)),
            end_vel=np.zeros((3, 1)))
        self.assertEqual(ppose.get_number_of_segments(), 2)
        # Ensure we can copy.
        self.assertEqual(copy.copy(ppose).get_number_of_segments(), 2)
        self.assertEqual(copy.deepcopy(ppose).get_number_of_segments(), 2)

    @numpy_compare.check_all_types
    def test_stacked_trajectory(self, T):
        breaks = [0, 1, 2]
        samples = [[[0]], [[1]], [[2]]]
        zoh = PiecewisePolynomial_[T].ZeroOrderHold(breaks, samples)
        dut = StackedTrajectory_[T](rowwise=True)
        dut.Append(zoh)
        dut.Append(zoh)
        self.assertEqual(dut.rows(), 2)
        self.assertEqual(dut.cols(), 1)
        dut.Clone()
        copy.copy(dut)
        copy.deepcopy(dut)
