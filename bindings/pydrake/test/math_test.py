from __future__ import absolute_import, division, print_function

import pydrake.math as mut
from pydrake.math import (BarycentricMesh, wrap_to)
from pydrake.util.eigen_geometry import Quaternion

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

    def test_rotation_matrix(self):
        R = mut.RotationMatrix()
        self.assertTrue(np.allclose(R.matrix(), np.eye(3)))
        self.assertTrue(np.allclose(
            mut.RotationMatrix.Identity().matrix(), np.eye(3)))
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
        rpy = mut.RollPitchYaw(rpy=[0, 0, 0])
        self.assertTrue(np.allclose(rpy.vector(), [0, 0, 0]))
        rpy = mut.RollPitchYaw(roll=0, pitch=0, yaw=0)
        self.assertTupleEqual(
            (rpy.roll_angle(), rpy.pitch_angle(), rpy.yaw_angle()),
            (0, 0, 0))
        q_I = Quaternion()
        self.assertTrue(np.allclose(rpy.ToQuaternion().wxyz(), q_I.wxyz()))
