from __future__ import absolute_import, division, print_function

import unittest
import numpy as np
from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.ipopt import IpoptSolver


class TestIpoptSolver(unittest.TestCase):
    def test_ipopt_solver(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint(x[0] >= 1)
        prog.AddLinearConstraint(x[1] >= 1)
        prog.AddQuadraticCost(np.eye(2), np.zeros(2), x)
        solver = IpoptSolver()
        self.assertTrue(solver.available())
        self.assertEqual(solver.solver_id().name(), "IPOPT")
        self.assertEqual(solver.SolverName(), "IPOPT")
        self.assertEqual(solver.solver_type(), mp.SolverType.kIpopt)
        result = solver.Solve(prog)
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)
        x_expected = np.array([1, 1])
        self.assertTrue(np.allclose(prog.GetSolution(x), x_expected))

    def unavailable(self):
        """Per the BUILD file, this test is only run when IPOPT is disabled."""
        solver = IpoptSolver()
        self.assertFalse(solver.available())


if __name__ == '__main__':
    unittest.main()
