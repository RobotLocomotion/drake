import numpy as np
import unittest

from pydrake.solvers import (
    CreateDualConvexProgram,
    MathematicalProgram,
)
from pydrake.symbolic import Variables


class TestDualConvexProgram(unittest.TestCase):
    def test_CreateDualConvexProgram(self):
        prog = MathematicalProgram()
        y = prog.NewContinuousVariables(2, "y")
        A = np.array([[0.5, 0.7], [-.2, 0.4], [-2.3, -4.5]])
        lb = np.array([1.3, -.24, 0.25])
        ub = np.array([5.6, 0.1, 1.4])
        prog.AddLinearConstraint(A, lb, ub, y)
        dual_prog = CreateDualConvexProgram(prog=prog)

        self.assertEqual(dual_prog.num_vars(), 6)
        self.assertEqual(len(dual_prog.linear_equality_constraints()),
                         1)
        self.assertEqual(len(dual_prog.linear_constraints(), 1))