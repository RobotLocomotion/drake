from __future__ import print_function, absolute_import

import unittest
import numpy as np
import pydrake
from pydrake.solvers import mathematicalprogram


class TestMathematicalProgram(unittest.TestCase):
    def test_program_construction(self):
        prog = mathematicalprogram.MathematicalProgram()
        vars = prog.NewContinuousVariables(5, "x")
        print(vars)

    def test_mixed_integer_optimization(self):
        prog = mathematicalprogram.MathematicalProgram()
        x = prog.NewBinaryVariables(3, "x")
        c = np.array([-1.0, -1.0, -2.0])
        prog.AddLinearCost(c, x)
        a = np.array([[1.0, 2.0, 3.0]])
        prog.AddLinearConstraint(a, -np.inf, 4, x)
        a2 = np.array([1.0, 1.0])
        prog.AddLinearConstraint(a2, 1, np.inf, x[:2])
        x_expected = np.array([1.0, 0.0, 1.0])
        result = prog.Solve()
        self.assertEqual(result, mathematicalprogram.SolutionResult.kSolutionFound)

        # Test that we got the right solution for all x
        print("got", prog.GetSolution(x))
        self.assertTrue(np.all(np.isclose(prog.GetSolution(x), x_expected)))

        # Also test by asking for the value of each element of x
        for i in range(3):
            self.assertAlmostEqual(prog.GetSolution(x[i]), x_expected[i])




if __name__ == '__main__':
    unittest.main()
