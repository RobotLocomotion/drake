import unittest

import numpy as np

from pydrake.solvers import ReplaceBilinearTerms
import pydrake.symbolic as sym


class TestReplaceBilinearTerms(unittest.TestCase):
    def test(self):
        x = np.array([sym.Variable("x0"), sym.Variable("x1")])
        y = np.array([
            sym.Variable("y0"), sym.Variable("y1"), sym.Variable("y2")])
        W = np.empty((2, 3), dtype=object)
        for i in range(2):
            for j in range(3):
                W[i, j] = sym.Variable(f"W({i}, {j})")
        e = x[0]*y[1] * 3 + x[1]*y[2] * 4
        e_replace = ReplaceBilinearTerms(e=e, x=x, y=y, W=W)
        self.assertTrue(e_replace.EqualTo(W[0, 1]*3 + W[1, 2]*4))
