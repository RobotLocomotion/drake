from __future__ import absolute_import, division, print_function

import pydrake.math as mut
import pydrake.math._test as mtest
from pydrake.math import (BarycentricMesh, wrap_to)
from pydrake.common.eigen_geometry import Isometry3_, Quaternion_, AngleAxis_
from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
import pydrake.common.test_utilities.numpy_compare as numpy_compare

import copy
import math
import unittest

import numpy as np
import six


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

    def test_wrap_to(self):
        self.assertEqual(wrap_to(1.5, 0., 1.), .5)


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
        X_I = np.eye(4)
        check_equality(RigidTransform(), X_I)
        check_equality(RigidTransform(other=RigidTransform()), X_I)
        check_equality(copy.copy(RigidTransform()), X_I)
        R_I = RotationMatrix()
        p_I = np.zeros(3)
        rpy_I = RollPitchYaw(0, 0, 0)
        quaternion_I = Quaternion.Identity()
        angle = np.pi * 0
        axis = [0, 0, 1]
        angle_axis = AngleAxis(angle=angle, axis=axis)
        check_equality(RigidTransform(R=R_I, p=p_I), X_I)
        check_equality(RigidTransform(rpy=rpy_I, p=p_I), X_I)
        check_equality(RigidTransform(quaternion=quaternion_I, p=p_I), X_I)
        check_equality(RigidTransform(theta_lambda=angle_axis, p=p_I), X_I)
        check_equality(RigidTransform(R=R_I), X_I)
        check_equality(RigidTransform(p=p_I), X_I)
        # - Accessors, mutators, and general methods.
        X = RigidTransform()
        X.set(R=R_I, p=p_I)
        X.SetFromIsometry3(pose=Isometry3.Identity())
        check_equality(RigidTransform.Identity(), X_I)
        self.assertIsInstance(X.rotation(), RotationMatrix)
        X.set_rotation(R=R_I)
        self.assertIsInstance(X.translation(), np.ndarray)
        X.set_translation(p=np.zeros(3))
        numpy_compare.assert_float_equal(X.GetAsMatrix4(), X_I)
        numpy_compare.assert_float_equal(X.GetAsMatrix34(), X_I[:3])
        self.assertIsInstance(X.GetAsIsometry3(), Isometry3)
        check_equality(X.inverse(), X_I)
        self.assertIsInstance(
            X.multiply(other=RigidTransform()), RigidTransform)
        self.assertIsInstance(X.multiply(p_BoQ_B=p_I), np.ndarray)
        if six.PY3:
            self.assertIsInstance(
                eval("X @ RigidTransform()"), RigidTransform)
            self.assertIsInstance(eval("X @ [0, 0, 0]"), np.ndarray)

    @numpy_compare.check_all_types
    def test_isometry_implicit(self, T):
        Isometry3 = Isometry3_[T]
        # Explicitly disabled, to mirror C++ API.
        with self.assertRaises(TypeError):
            self.assertTrue(mtest.TakeRigidTransform(Isometry3()))
        self.assertTrue(mtest.TakeIsometry3(mut.RigidTransform()))

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
        R.set(R=np.eye(3))
        numpy_compare.assert_float_equal(R.matrix(), np.eye(3))
        # - Nontrivial quaternion.
        q = Quaternion(wxyz=[0.5, 0.5, 0.5, 0.5])
        R = RotationMatrix(quaternion=q)
        q_R = R.ToQuaternion()
        numpy_compare.assert_float_equal(
            q.wxyz(), numpy_compare.to_float(q_R.wxyz()))
        # - Inverse, transpose, projection
        R_I = R.inverse().multiply(R)
        numpy_compare.assert_float_equal(R_I.matrix(), np.eye(3))
        R_T = R.transpose().multiply(R)
        numpy_compare.assert_float_equal(R_T.matrix(), np.eye(3))
        R_P = RotationMatrix.ProjectToRotationMatrix(M=2*np.eye(3))
        numpy_compare.assert_float_equal(R_P.matrix(), np.eye(3))
        # Matrix checks
        numpy_compare.assert_equal(R.IsValid(), True)
        R = RotationMatrix()
        numpy_compare.assert_equal(R.IsExactlyIdentity(), True)
        numpy_compare.assert_equal(R.IsIdentityToInternalTolerance(), True)
        if six.PY3:
            numpy_compare.assert_float_equal(
                    eval("R.inverse() @ R").matrix(), np.eye(3))

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

    def test_orthonormal_basis(self):
        R = mut.ComputeBasisFromAxis(axis_index=0, axis_W=[1, 0, 0])
        self.assertAlmostEqual(np.linalg.det(R), 1.0)
        self.assertTrue(np.allclose(R.dot(R.T), np.eye(3)))

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
