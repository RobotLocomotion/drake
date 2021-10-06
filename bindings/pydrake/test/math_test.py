import pydrake.math as mut
from pydrake.math import (BarycentricMesh, wrap_to)
from pydrake.common import RandomGenerator
from pydrake.common.cpp_param import List
from pydrake.common.eigen_geometry import Isometry3_, Quaternion_, AngleAxis_
from pydrake.common.value import Value
from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
import pydrake.common.test_utilities.numpy_compare as numpy_compare
from pydrake.common.test_utilities.pickle_compare import assert_pickle

import copy
import math
import textwrap
import unittest

import numpy as np


class TestBarycentricMesh(unittest.TestCase):
    def test_spelling(self):
        mesh = BarycentricMesh([{0, 1}, {0, 1}])
        values = np.array([[0, 1, 2, 3]])
        grid = mesh.get_input_grid()
        self.assertIsInstance(grid, list)
        self.assertEqual(len(grid), 2)
        self.assertIsInstance(grid[0], set)
        self.assertEqual(len(grid[0]), 2)
        self.assertEqual(mesh.get_input_size(), 2)
        self.assertEqual(mesh.get_num_mesh_points(), 4)
        self.assertEqual(mesh.get_num_interpolants(), 3)
        self.assertTrue((mesh.get_mesh_point(0) == [0., 0.]).all())
        points = mesh.get_all_mesh_points()
        self.assertEqual(points.shape, (2, 4))
        self.assertTrue((points[:, 3] == [1., 1.]).all())
        self.assertEqual(mesh.Eval(values, (0, 0))[0], 0)
        self.assertEqual(mesh.Eval(values, (1, 0))[0], 1)
        self.assertEqual(mesh.Eval(values, (0, 1))[0], 2)
        self.assertEqual(mesh.Eval(values, (1, 1))[0], 3)

    def test_weight(self):
        mesh = BarycentricMesh([{0, 1}, {0, 1}])

        (Ti, T) = mesh.EvalBarycentricWeights((0., 1.))
        np.testing.assert_equal(Ti, [2, 2, 0])
        np.testing.assert_almost_equal(T, (1., 0., 0.))

    def test_mesh_values_from(self):
        mesh = BarycentricMesh([{0, 1}, {0, 1}])

        def mynorm(x):
            return [x.dot(x)]

        values = mesh.MeshValuesFrom(mynorm)
        self.assertEqual(values.size, 4)


# TODO(eric.cousineau): Test wrappings against non-identity transforms.


class TestMath(unittest.TestCase):
    def test_math(self):
        # Compare against `math` functions.
        # TODO(eric.cousineau): Consider removing this and only rely on
        # `math_overloads_test`, which already tests this.
        unary = [
            (mut.log, math.log),
            (mut.abs, math.fabs),
            (mut.exp, math.exp),
            (mut.sqrt, math.sqrt),
            (mut.sin, math.sin),
            (mut.cos, math.cos),
            (mut.tan, math.tan),
            (mut.asin, math.asin),
            (mut.acos, math.acos),
            (mut.atan, math.atan),
            (mut.sinh, math.sinh),
            (mut.cosh, math.cosh),
            (mut.tanh, math.tanh),
            (mut.ceil, math.ceil),
            (mut.floor, math.floor),
        ]
        binary = [
            (mut.min, min),
            (mut.max, max),
            (mut.pow, math.pow),
            (mut.atan2, math.atan2),
        ]

        a = 0.1
        b = 0.2
        for f_core, f_cpp in unary:
            self.assertEqual(f_core(a), f_cpp(a), (f_core, f_cpp))
        for f_core, f_cpp in binary:
            self.assertEqual(f_core(a, b), f_cpp(a, b))

    def check_cast(self, template, T):
        value = template[T]()
        # Refer to docstrings for `CastUPack` in `default_scalars_pybind.h`.
        if T == float:
            U_list = [float, AutoDiffXd, Expression]
        else:
            U_list = [T]
        for U in U_list:
            self.assertIsInstance(value.cast[U](), template[U], U)

    @numpy_compare.check_all_types
    def test_rigid_transform(self, T):
        RigidTransform = mut.RigidTransform_[T]
        RotationMatrix = mut.RotationMatrix_[T]
        RollPitchYaw = mut.RollPitchYaw_[T]
        Isometry3 = Isometry3_[T]
        Quaternion = Quaternion_[T]
        AngleAxis = AngleAxis_[T]

        def check_equality(X_actual, X_expected_matrix):
            self.assertIsInstance(X_actual, RigidTransform)
            numpy_compare.assert_float_equal(
                    X_actual.GetAsMatrix4(), X_expected_matrix)

        # - Constructors.
        X_I_np = np.eye(4)
        check_equality(RigidTransform(), X_I_np)
        check_equality(RigidTransform(other=RigidTransform()), X_I_np)
        check_equality(copy.copy(RigidTransform()), X_I_np)
        R_I = RotationMatrix()
        p_I = np.zeros(3)
        rpy_I = RollPitchYaw(0, 0, 0)
        quaternion_I = Quaternion.Identity()
        angle = np.pi * 0
        axis = [0, 0, 1]
        angle_axis = AngleAxis(angle=angle, axis=axis)
        check_equality(RigidTransform(R=R_I, p=p_I), X_I_np)
        check_equality(RigidTransform(rpy=rpy_I, p=p_I), X_I_np)
        check_equality(RigidTransform(quaternion=quaternion_I, p=p_I), X_I_np)
        check_equality(RigidTransform(theta_lambda=angle_axis, p=p_I), X_I_np)
        check_equality(RigidTransform(R=R_I), X_I_np)
        check_equality(RigidTransform(p=p_I), X_I_np)
        check_equality(RigidTransform(pose=p_I), X_I_np)
        check_equality(RigidTransform(pose=X_I_np), X_I_np)
        check_equality(RigidTransform(pose=X_I_np[:3]), X_I_np)
        # - Cast.
        self.check_cast(mut.RigidTransform_, T)
        # - Accessors, mutators, and general methods.
        X = RigidTransform()
        X.set(R=R_I, p=p_I)
        X.SetFromIsometry3(pose=Isometry3.Identity())
        check_equality(RigidTransform.Identity(), X_I_np)
        self.assertIsInstance(X.rotation(), RotationMatrix)
        X.set_rotation(R=R_I)
        X.set_rotation(rpy=rpy_I)
        X.set_rotation(quaternion=quaternion_I)
        X.set_rotation(theta_lambda=angle_axis)
        self.assertIsInstance(X.translation(), np.ndarray)
        X.set_translation(p=np.zeros(3))
        numpy_compare.assert_float_equal(X.GetAsMatrix4(), X_I_np)
        numpy_compare.assert_float_equal(X.GetAsMatrix34(), X_I_np[:3])
        self.assertIsInstance(X.GetAsIsometry3(), Isometry3)
        check_equality(X.inverse(), X_I_np)
        self.assertIsInstance(
            X.multiply(other=RigidTransform()), RigidTransform)
        self.assertIsInstance(X @ RigidTransform(), RigidTransform)
        self.assertIsInstance(X @ [0, 0, 0], np.ndarray)
        if T != Expression:
            self.assertTrue(X.IsExactlyIdentity())
            self.assertTrue(X.IsIdentityToEpsilon(translation_tolerance=0))
            self.assertTrue(X.IsNearlyEqualTo(other=X, tolerance=0))
        # - Test shaping (#13885).
        v = np.array([0., 0., 0.])
        vs = np.array([[1., 2., 3.], [4., 5., 6.]]).T
        self.assertEqual((X @ v).shape, (3,))
        self.assertEqual((X @ v.reshape((3, 1))).shape, (3, 1))
        self.assertEqual((X @ vs).shape, (3, 2))
        # - Test vector multiplication.
        R_AB = RotationMatrix([
            [0., 1, 0],
            [-1, 0, 0],
            [0, 0, 1]])
        p_AB = np.array([1., 2, 3])
        X_AB = RigidTransform(R=R_AB, p=p_AB)
        p_BQ = [10, 20, 30]
        p_AQ = [21., -8, 33]
        numpy_compare.assert_float_equal(X_AB.multiply(p_BoQ_B=p_BQ), p_AQ)
        # N.B. Remember that this takes ndarray[3, n], NOT ndarray[n, 3]!
        p_BQlist = np.array([p_BQ, p_BQ]).T
        p_AQlist = np.array([p_AQ, p_AQ]).T
        numpy_compare.assert_float_equal(
            X_AB.multiply(p_BoQ_B=p_BQlist), p_AQlist)
        # - Repr.
        z = repr(T(0.0))
        i = repr(T(1.0))
        type_suffix = {
            float: "",
            AutoDiffXd: "_[AutoDiffXd]",
            Expression: "_[Expression]",
        }[T]
        self.assertEqual(repr(RigidTransform()), textwrap.dedent(f"""\
        RigidTransform{type_suffix}(
          R=RotationMatrix{type_suffix}([
            [{i}, {z}, {z}],
            [{z}, {i}, {z}],
            [{z}, {z}, {i}],
          ]),
          p=[{z}, {z}, {z}],
        )"""))
        if T == float:
            # TODO(jwnimmer-tri) Once AutoDiffXd and Expression implement an
            # eval-able repr, then we can test more than just T=float here.
            roundtrip = eval(repr(RigidTransform()))
            # TODO(jwnimmer-tri) Once IsExactlyEqualTo is bound, we can easily
            # check the contents of the roundtrip object here.
            self.assertIsInstance(roundtrip, RigidTransform)
        # Test pickling.
        assert_pickle(self, X_AB, RigidTransform.GetAsMatrix4, T=T)

    @numpy_compare.check_all_types
    def test_rotation_matrix(self, T):
        # - Constructors.
        RotationMatrix = mut.RotationMatrix_[T]
        AngleAxis = AngleAxis_[T]
        Quaternion = Quaternion_[T]
        RollPitchYaw = mut.RollPitchYaw_[T]

        R = RotationMatrix()
        numpy_compare.assert_float_equal(
                RotationMatrix(other=R).matrix(), np.eye(3))
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        numpy_compare.assert_float_equal(copy.copy(R).matrix(), np.eye(3))
        numpy_compare.assert_float_equal(
                RotationMatrix.Identity().matrix(), np.eye(3))
        R = RotationMatrix(R=np.eye(3))
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        R = RotationMatrix(quaternion=Quaternion.Identity())
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        R = RotationMatrix(theta_lambda=AngleAxis(angle=0, axis=[0, 0, 1]))
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        R = RotationMatrix(rpy=RollPitchYaw(rpy=[0, 0, 0]))
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        # One axis RotationMatrices
        R = RotationMatrix.MakeXRotation(theta=0)
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        R = RotationMatrix.MakeYRotation(theta=0)
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        R = RotationMatrix.MakeZRotation(theta=0)
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        # TODO(eric.cousineau): #11575, remove the conditional.
        if T == float:
            numpy_compare.assert_float_equal(R.row(index=0), [1., 0., 0.])
            numpy_compare.assert_float_equal(R.col(index=0), [1., 0., 0.])
            R = RotationMatrix.MakeFromOneVector(b_A=[1, 0, 0], axis_index=0)
            numpy_compare.assert_equal(R.IsValid(), True)
        R.set(R=np.eye(3))
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        # - Cast.
        self.check_cast(mut.RotationMatrix_, T)
        # - Nontrivial quaternion.
        q = Quaternion(wxyz=[0.5, 0.5, 0.5, 0.5])
        R = RotationMatrix(quaternion=q)
        q_R = R.ToQuaternion()
        numpy_compare.assert_float_equal(
            q.wxyz(), numpy_compare.to_float(q_R.wxyz()))
        # - Conversion to AngleAxis
        angle_axis = R.ToAngleAxis()
        self.assertIsInstance(angle_axis, AngleAxis)
        R_AngleAxis = RotationMatrix(angle_axis)
        R_I = R.inverse().multiply(R_AngleAxis)
        numpy_compare.assert_equal(R_I.IsIdentityToInternalTolerance(), True)
        # - Inverse, transpose, projection
        R_I = R.inverse().multiply(R)
        numpy_compare.assert_float_equal(R_I.matrix(), np.eye(3))
        numpy_compare.assert_float_equal((R.inverse() @ R).matrix(), np.eye(3))
        R_T = R.transpose().multiply(R)
        numpy_compare.assert_float_equal(R_T.matrix(), np.eye(3))
        R_P = RotationMatrix.ProjectToRotationMatrix(M=2*np.eye(3))
        numpy_compare.assert_float_equal(R_P.matrix(), np.eye(3))
        # - Multiplication.
        R_AB = RotationMatrix([
            [0., 1, 0],
            [-1, 0, 0],
            [0, 0, 1]])
        v_B = [10, 20, 30]
        v_A = [20., -10., 30]
        numpy_compare.assert_float_equal(R_AB.multiply(v_B=v_B), v_A)
        # N.B. Remember that this takes ndarray[3, n], NOT ndarray[n, 3]!
        vlist_B = np.array([v_B, v_B]).T
        vlist_A = np.array([v_A, v_A]).T
        numpy_compare.assert_float_equal(R_AB.multiply(v_B=vlist_B), vlist_A)
        # - Test shaping (#13885).
        v = np.array([0., 0., 0.])
        vs = np.array([[1., 2., 3.], [4., 5., 6.]]).T
        self.assertEqual((R_AB @ v).shape, (3,))
        self.assertEqual((R_AB @ v.reshape((3, 1))).shape, (3, 1))
        self.assertEqual((R_AB @ vs).shape, (3, 2))
        # Matrix checks
        numpy_compare.assert_equal(R.IsValid(), True)
        R = RotationMatrix()
        numpy_compare.assert_equal(R.IsExactlyIdentity(), True)
        numpy_compare.assert_equal(R.IsIdentityToInternalTolerance(), True)
        # - Repr.
        z = repr(T(0.0))
        i = repr(T(1.0))
        type_suffix = {
            float: "",
            AutoDiffXd: "_[AutoDiffXd]",
            Expression: "_[Expression]",
        }[T]
        self.assertEqual(repr(RotationMatrix()), textwrap.dedent(f"""\
        RotationMatrix{type_suffix}([
          [{i}, {z}, {z}],
          [{z}, {i}, {z}],
          [{z}, {z}, {i}],
        ])"""))
        if T == float:
            # TODO(jwnimmer-tri) Once AutoDiffXd and Expression implement an
            # eval-able repr, then we can test more than just T=float here.
            roundtrip = eval(repr(RotationMatrix()))
            self.assertTrue(roundtrip.IsExactlyIdentity())
        # Test pickling.
        assert_pickle(self, R_AB, RotationMatrix.matrix, T=T)

    @numpy_compare.check_all_types
    def test_roll_pitch_yaw(self, T):
        # - Constructors.
        RollPitchYaw = mut.RollPitchYaw_[T]
        RotationMatrix = mut.RotationMatrix_[T]
        Quaternion = Quaternion_[T]

        rpy = RollPitchYaw(rpy=[0, 0, 0])
        numpy_compare.assert_float_equal(
                RollPitchYaw(other=rpy).vector(), [0., 0., 0.])
        numpy_compare.assert_float_equal(rpy.vector(), [0., 0., 0.])
        rpy = RollPitchYaw(roll=0, pitch=0, yaw=0)
        numpy_compare.assert_float_equal([
            rpy.roll_angle(), rpy.pitch_angle(), rpy.yaw_angle()],
            [0., 0., 0.])
        rpy = RollPitchYaw(R=RotationMatrix())
        numpy_compare.assert_float_equal(rpy.vector(), [0., 0., 0.])
        rpy = RollPitchYaw(matrix=np.eye(3))
        numpy_compare.assert_float_equal(rpy.vector(), [0., 0., 0.])
        q_I = Quaternion()
        rpy_q_I = RollPitchYaw(quaternion=q_I)
        numpy_compare.assert_float_equal(rpy_q_I.vector(), [0., 0., 0.])
        # - Additional properties.
        numpy_compare.assert_float_equal(
                rpy.ToQuaternion().wxyz(), numpy_compare.to_float(q_I.wxyz()))
        R = rpy.ToRotationMatrix().matrix()
        numpy_compare.assert_float_equal(R, np.eye(3))
        # - Converting changes in orientation
        numpy_compare.assert_float_equal(rpy.CalcRotationMatrixDt(
            rpyDt=[0, 0, 0]), np.zeros((3, 3)))
        numpy_compare.assert_float_equal(
            rpy.CalcAngularVelocityInParentFromRpyDt(rpyDt=[0, 0, 0]),
            [0., 0., 0.])
        numpy_compare.assert_float_equal(
            rpy.CalcAngularVelocityInChildFromRpyDt(rpyDt=[0, 0, 0]),
            [0., 0., 0.])
        numpy_compare.assert_float_equal(
            rpy.CalcRpyDtFromAngularVelocityInParent(w_AD_A=[0, 0, 0]),
            [0., 0., 0.])
        numpy_compare.assert_float_equal(
            rpy.CalcRpyDDtFromRpyDtAndAngularAccelInParent(
                rpyDt=[0, 0, 0], alpha_AD_A=[0, 0, 0]), [0., 0., 0.])
        numpy_compare.assert_float_equal(rpy.CalcRpyDDtFromAngularAccelInChild(
            rpyDt=[0, 0, 0], alpha_AD_D=[0, 0, 0]), [0., 0., 0.])
        # Test pickling.
        assert_pickle(self, rpy, RollPitchYaw.vector, T=T)

    @numpy_compare.check_all_types
    def test_bspline_basis(self, T):
        BsplineBasis = mut.BsplineBasis_[T]

        bspline = BsplineBasis()
        self.assertEqual(bspline.order(), 0)
        self.assertEqual(BsplineBasis(other=bspline).order(), 0)
        bspline = BsplineBasis(order=2, knots=[0, 1, 3, 5])
        self.assertEqual(bspline.order(), 2)
        bspline = BsplineBasis(order=2, num_basis_functions=3,
                               type=mut.KnotVectorType.kUniform,
                               initial_parameter_value=5.,
                               final_parameter_value=6.)
        self.assertEqual(bspline.order(), 2)
        self.assertEqual(bspline.degree(), 1)
        self.assertEqual(bspline.num_basis_functions(), 3)
        numpy_compare.assert_float_equal(bspline.knots(),
                                         [4.5, 5.0, 5.5, 6.0, 6.5])
        numpy_compare.assert_float_equal(bspline.initial_parameter_value(), 5.)
        numpy_compare.assert_float_equal(bspline.final_parameter_value(), 6.)
        self.assertEqual(
            bspline.FindContainingInterval(parameter_value=5.2), 1)
        self.assertEqual(
            bspline.ComputeActiveBasisFunctionIndices(
                parameter_interval=[5.2, 5.7]),
            [0, 1, 2])
        self.assertEqual(
            bspline.ComputeActiveBasisFunctionIndices(parameter_value=5.4),
            [0, 1])
        numpy_compare.assert_float_equal(
            bspline.EvaluateBasisFunctionI(i=0, parameter_value=5.7), 0.)

    @numpy_compare.check_all_types
    def test_wrap_to(self, T):
        value = wrap_to(T(1.5), T(0.), T(1.))
        if T != Expression:
            self.assertEqual(value, T(.5))

    def test_random_rotations(self):
        g = RandomGenerator()
        quat = mut.UniformlyRandomQuaternion(g)
        self.assertIsInstance(quat, Quaternion_[float])
        angle_axis = mut.UniformlyRandomAngleAxis(g)
        self.assertIsInstance(angle_axis, AngleAxis_[float])
        rot_mat = mut.UniformlyRandomRotationMatrix(g)
        self.assertIsInstance(rot_mat, mut.RotationMatrix)
        rpy = mut.UniformlyRandomRPY(g)
        self.assertIsInstance(rpy, np.ndarray)
        self.assertEqual(len(rpy), 3)

    def test_matrix_util(self):
        A = np.array([[1, 2], [3, 4]])

        self.assertFalse(mut.IsSymmetric(matrix=A))
        self.assertFalse(mut.IsSymmetric(matrix=A, precision=0))
        self.assertTrue(mut.IsSymmetric(np.eye(3), 0.))

        self.assertFalse(mut.IsPositiveDefinite(matrix=A, tolerance=0))
        self.assertTrue(mut.IsPositiveDefinite(A.dot(A.T)))

    def test_quadratic_form(self):
        Q = np.diag([1., 2., 3.])
        X = mut.DecomposePSDmatrixIntoXtransposeTimesX(Q, 1e-8)
        np.testing.assert_array_almost_equal(X, np.sqrt(Q))
        b = np.zeros(3)
        c = 4.
        R, d = mut.DecomposePositiveQuadraticForm(Q, b, c)
        self.assertEqual(np.size(R, 0), 4)
        self.assertEqual(np.size(R, 1), 3)
        self.assertEqual(len(d), 4)
        T = mut.BalanceQuadraticForms(S=np.eye(3), P=np.eye(3))
        np.testing.assert_array_almost_equal(T, np.eye(3))

    def test_riccati_lyapunov(self):
        A = 0.1*np.eye(2)
        B = np.eye(2)
        Q = np.eye(2)
        R = np.eye(2)

        mut.ContinuousAlgebraicRiccatiEquation(A=A, B=B, Q=Q, R=R)
        mut.RealContinuousLyapunovEquation(A=A, Q=Q)
        mut.RealDiscreteLyapunovEquation(A=A, Q=Q)

        A = np.array([[1, 1], [0, 1]])
        B = np.array([[0], [1]])
        Q = np.array([[1, 0], [0, 0]])
        R = [0.3]

        mut.DiscreteAlgebraicRiccatiEquation(A=A, B=B, Q=Q, R=R)

    @numpy_compare.check_all_types
    def test_value_instantiations(self, T):
        # Existence checks.
        Value[mut.RigidTransform_[T]]
        Value[List[mut.RigidTransform_[T]]]
        Value[mut.RotationMatrix_[T]]
        Value[List[mut.RotationMatrix_[T]]]
