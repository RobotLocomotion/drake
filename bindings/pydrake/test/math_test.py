from __future__ import absolute_import, division, print_function

import pydrake.math as mut
from pydrake.math import (BarycentricMesh, wrap_to)

import unittest
import numpy as np

import math


class TestBarycentricMesh(unittest.TestCase):
    def test_spelling(self):
        mesh = BarycentricMesh([{0, 1}, {0, 1}])
        values = np.array([[0, 1, 2, 3]])
        grid = mesh.get_input_grid()
        self.assertIsInstance(grid, list)
        self.assertEquals(len(grid), 2)
        self.assertIsInstance(grid[0], set)
        self.assertEquals(len(grid[0]), 2)
        self.assertEquals(mesh.get_input_size(), 2)
        self.assertEquals(mesh.get_num_mesh_points(), 4)
        self.assertEquals(mesh.get_num_interpolants(), 3)
        self.assertTrue((mesh.get_mesh_point(0) == [0., 0.]).all())
        points = mesh.get_all_mesh_points()
        self.assertEqual(points.shape, (2, 4))
        self.assertTrue((points[:, 3] == [1., 1.]).all())
        self.assertEquals(mesh.Eval(values, (0, 0))[0], 0)
        self.assertEquals(mesh.Eval(values, (1, 0))[0], 1)
        self.assertEquals(mesh.Eval(values, (0, 1))[0], 2)
        self.assertEquals(mesh.Eval(values, (1, 1))[0], 3)

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
        self.assertEquals(values.size, 4)

    def test_wrap_to(self):
        self.assertEquals(wrap_to(1.5, 0., 1.), .5)

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
            self.assertEquals(f_core(a), f_cpp(a), (f_core, f_cpp))
        for f_core, f_cpp in binary:
            self.assertEquals(f_core(a, b), f_cpp(a, b))
