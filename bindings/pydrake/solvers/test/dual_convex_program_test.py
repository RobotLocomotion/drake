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
        linear_constraint = prog.AddLinearConstraint(A, lb, ub, y)
        dual_prog, constraint_to_dual_vars = CreateDualConvexProgram(prog=prog)

        self.assertEqual(dual_prog.num_vars(), 6)
        self.assertEqual(len(dual_prog.linear_equality_constraints()),
                         1)
        self.assertEqual(len(dual_prog.linear_constraints()), 1)
        print(constraint_to_dual_vars)
        print(linear_constraint)
        print(list(constraint_to_dual_vars.keys())[0])
        print(constraint_to_dual_vars[linear_constraint])
        self.assertTrue(linear_constraint in constraint_to_dual_vars.keys())
