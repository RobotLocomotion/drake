from __future__ import print_function, absolute_import

import unittest
import numpy as np
import pydrake
from pydrake.solvers import mathematicalprogram as mp


class TestMathematicalProgram(unittest.TestCase):
    def test_program_construction(self):
        prog = mp.MathematicalProgram()
        vars = prog.NewContinuousVariables(5, "x")

    def test_mixed_integer_optimization(self):
        prog = mp.MathematicalProgram()
        x = prog.NewBinaryVariables(3, "x")
        c = np.array([-1.0, -1.0, -2.0])
        prog.AddLinearCost(c, x)
        a = np.array([[1.0, 2.0, 3.0]])
        prog.AddLinearConstraint(a.dot(x)[0], -np.inf, 4)
        prog.AddLinearConstraint(x[0] + x[1], 1, np.inf)
        result = prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)

        # Test that we got the right solution for all x
        x_expected = np.array([1.0, 0.0, 1.0])
        self.assertTrue(np.all(np.isclose(prog.GetSolution(x), x_expected)))

        # Also test by asking for the value of each element of x
        for i in range(3):
            self.assertAlmostEqual(prog.GetSolution(x[i]), x_expected[i])

    def test_expression_constraints(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        ex = mp.Expression(x[0]) + x[1]
        prog.AddLinearConstraint(ex, -1, 1)






if __name__ == '__main__':
    unittest.main()
