from __future__ import absolute_import, division, print_function

import unittest
import numpy as np
from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.mosek import MosekSolver


class TestMathematicalProgram(unittest.TestCase):
    def test_mosek_solver(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint(x[0] >= 1)
        prog.AddLinearConstraint(x[1] >= 1)
        prog.AddQuadraticCost(np.eye(2), np.zeros(2), x)
        solver = MosekSolver()
        self.assertTrue(solver.available())
        self.assertEqual(solver.solver_type(), mp.SolverType.kMosek)
        result = solver.Solve(prog)
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)
        x_expected = np.array([1, 1])
        self.assertTrue(np.allclose(prog.GetSolution(x), x_expected))


if __name__ == '__main__':
    unittest.main()
