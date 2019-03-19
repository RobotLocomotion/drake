import pydrake.visualization.plotting as mut

import matplotlib.pyplot as plt
import numpy as np
import unittest

from pydrake.symbolic import Variable


class TestMatplotlibUtil(unittest.TestCase):
    def test_plot_sublevelset_quadratic(self):
        fig, ax = plt.subplots()

        A = np.diag([1., 2.])
        b = [3., 4.]
        c = -5.
        vertices = 11
        facecolor = (0., 0., 1., 1.)
        polys = mut.plot_sublevelset_quadratic(ax=ax, A=A, b=b, c=c,
                                               vertices=vertices,
                                               facecolor=facecolor)
        x = polys[0].get_xy()
        self.assertEqual(np.size(x, 0), vertices+1)
        for i in range(vertices):
            y = x[i, :]
            val = y.dot(A).dot(y) + y.dot(b) + c
            np.testing.assert_almost_equal(val, 1.)

        self.assertEqual(polys[0].get_facecolor(), facecolor)

    def test_plot_sublevelset_expression(self):
        fig, ax = plt.subplots()

        x = np.array([Variable("x0"), Variable("x1")])
        A = np.diag([1., 2.])
        b = [3., 4.]
        c = -5.
        e = x.dot(A).dot(x) + x.dot(b) + c
        vertices = 11
        facecolor = (0., 0., 1., 1.)
        polys = mut.plot_sublevelset_expression(ax=ax, e=e, vertices=vertices,
                                                facecolor=facecolor)
        x = polys[0].get_xy()
        self.assertEqual(np.size(x, 0), vertices+1)
        for i in range(np.size(x, 0)):
            y = x[i, :]
            val = y.dot(A).dot(y) + y.dot(b) + c
            np.testing.assert_almost_equal(val, 1.)

        self.assertEqual(polys[0].get_facecolor(), facecolor)
