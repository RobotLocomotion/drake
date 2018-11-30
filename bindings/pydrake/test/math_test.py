from __future__ import absolute_import, division, print_function

import pydrake.math as mut
from pydrake.math import (BarycentricMesh, wrap_to)
from pydrake.util.eigen_geometry import Isometry3, Quaternion, AngleAxis

import unittest
import numpy as np

import math


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

    def test_rigid_transform(self):

        def check_equality(X_actual, X_expected_matrix):
            # TODO(eric.cousineau): Use `IsNearlyEqualTo`.
            self.assertIsInstance(X_actual, mut.RigidTransform)
            self.assertTrue(
                np.allclose(X_actual.GetAsMatrix4(), X_expected_matrix))

        # - Constructors.
        X_I = np.eye(4)
        check_equality(mut.RigidTransform(), X_I)
        R_I = mut.RotationMatrix()
        p_I = np.zeros(3)
        rpy_I = mut.RollPitchYaw(0, 0, 0)
        quaternion_I = Quaternion.Identity()
        angle = np.pi * 0
        axis = [0, 0, 1]
        angle_axis = AngleAxis(angle=angle, axis=axis)
        check_equality(mut.RigidTransform(R=R_I, p=p_I), X_I)
        check_equality(mut.RigidTransform(rpy=rpy_I, p=p_I), X_I)
        check_equality(mut.RigidTransform(quaternion=quaternion_I, p=p_I), X_I)
        check_equality(mut.RigidTransform(theta_lambda=angle_axis, p=p_I), X_I)
        check_equality(mut.RigidTransform(R=R_I), X_I)
        check_equality(mut.RigidTransform(p=p_I), X_I)
        # - Accessors, mutators, and general methods.
        X = mut.RigidTransform()
        X.set(R=R_I, p=p_I)
        X.SetFromIsometry3(pose=Isometry3.Identity())
        check_equality(mut.RigidTransform.Identity(), X_I)
        self.assertIsInstance(X.rotation(), mut.RotationMatrix)
        X.set_rotation(R=R_I)
        self.assertIsInstance(X.translation(), np.ndarray)
        X.set_translation(p=np.zeros(3))
        self.assertTrue(np.allclose(X.GetAsMatrix4(), X_I))
        self.assertTrue(np.allclose(X.GetAsMatrix34(), X_I[:3]))
        self.assertIsInstance(X.GetAsIsometry3(), Isometry3)
        check_equality(X.inverse(), X_I)
        self.assertIsInstance(
            X.multiply(other=mut.RigidTransform()), mut.RigidTransform)
        self.assertIsInstance(X.multiply(p_BoQ_B=p_I), np.ndarray)

    def test_rotation_matrix(self):
        # - Constructors.
        R = mut.RotationMatrix()
        self.assertTrue(np.allclose(R.matrix(), np.eye(3)))
        self.assertTrue(np.allclose(
            mut.RotationMatrix.Identity().matrix(), np.eye(3)))
        R = mut.RotationMatrix(R=np.eye(3))
        self.assertTrue(np.allclose(R.matrix(), np.eye(3)))
        R = mut.RotationMatrix(quaternion=Quaternion.Identity())
        self.assertTrue(np.allclose(R.matrix(), np.eye(3)))
        R = mut.RotationMatrix(rpy=mut.RollPitchYaw(rpy=[0, 0, 0]))
        self.assertTrue(np.allclose(R.matrix(), np.eye(3)))
        # - Nontrivial quaternion.
        q = Quaternion(wxyz=[0.5, 0.5, 0.5, 0.5])
        R = mut.RotationMatrix(quaternion=q)
        q_R = R.ToQuaternion()
        self.assertTrue(np.allclose(q.wxyz(), q_R.wxyz()))
        # - Inverse.
        R_I = R.inverse().multiply(R)
        self.assertTrue(np.allclose(R_I.matrix(), np.eye(3)))

    def test_roll_pitch_yaw(self):
        # - Constructors.
        rpy = mut.RollPitchYaw(rpy=[0, 0, 0])
        self.assertTrue(np.allclose(rpy.vector(), [0, 0, 0]))
        rpy = mut.RollPitchYaw(roll=0, pitch=0, yaw=0)
        self.assertTupleEqual(
            (rpy.roll_angle(), rpy.pitch_angle(), rpy.yaw_angle()),
            (0, 0, 0))
        rpy = mut.RollPitchYaw(R=mut.RotationMatrix())
        self.assertTrue(np.allclose(rpy.vector(), [0, 0, 0]))
        q_I = Quaternion()
        rpy_q_I = mut.RollPitchYaw(quaternion=q_I)
        self.assertTrue(np.allclose(rpy_q_I.vector(), [0, 0, 0]))
        # - Additional properties.
        self.assertTrue(np.allclose(rpy.ToQuaternion().wxyz(), q_I.wxyz()))
        R = rpy.ToRotationMatrix().matrix()
        self.assertTrue(np.allclose(R, np.eye(3)))

    def test_orthonormal_basis(self):
        R = mut.ComputeBasisFromAxis(axis_index=0, axis_W=[1, 0, 0])
        self.assertAlmostEqual(np.linalg.det(R), 1.0)
        self.assertTrue(np.allclose(R.dot(R.T), np.eye(3)))
