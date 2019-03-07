from __future__ import absolute_import, division, print_function

import unittest
import numpy as np
from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.osqp import OsqpSolver


class TestOsqpSolver(unittest.TestCase):
    def test_osqp_solver(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint(x[0] >= 1)
        prog.AddLinearConstraint(x[1] >= 1)
        prog.AddQuadraticCost(np.eye(2), np.zeros(2), x)
        solver = OsqpSolver()
        self.assertTrue(solver.available())
        self.assertEqual(solver.solver_type(), mp.SolverType.kOsqp)
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        x_expected = np.array([1, 1])
        self.assertTrue(np.allclose(result.GetSolution(x), x_expected))
        self.assertEqual(result.get_solver_details().status_val, 1)

    def unavailable(self):
        """Per the BUILD file, this test is only run when OSQP is disabled."""
        solver = OsqpSolver()
        self.assertFalse(solver.available())
