import unittest
import warnings

import numpy as np

from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.clp import ClpSolver


class TestClpSolver(unittest.TestCase):
    def _make_prog(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(4, "x")
        prog.AddLinearCost(-3*x[0] - 2*x[1])
        prog.AddLinearCost(x[1] - 5 * x[2] - x[3] + 2)
        prog.AddLinearConstraint(3*x[0] + x[1] + 2*x[2] == 30)
        prog.AddLinearConstraint(2*x[0] + x[1] + 3 * x[2] + x[3] >= 15)
        prog.AddLinearConstraint(2 * x[1] + 3 * x[3] <= 25)
        prog.AddLinearConstraint(
            np.array([[1, 2]]), [-100], [40], [x[0], x[2]])
        prog.AddBoundingBoxConstraint(0, np.inf, x)
        prog.AddLinearConstraint(x[1] <= 10)
        x_expected = np.array([0, 0, 15., 25./3])
        return prog, x, x_expected

    def test_clp_solver(self):
        prog, x, x_expected = self._make_prog()
        solver = ClpSolver()
        self.assertEqual(solver.solver_id(), ClpSolver.id())
        self.assertTrue(solver.available())
        self.assertEqual(solver.solver_id().name(), "CLP")
        self.assertEqual(solver.SolverName(), "CLP")
        self.assertEqual(solver.solver_type(), mp.SolverType.kClp)
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        self.assertTrue(np.allclose(result.GetSolution(x), x_expected))
        self.assertEqual(result.get_solver_details().status, 0)
        self.assertAlmostEqual(result.get_optimal_cost(), -244./3)

    def unavailable(self):
        """Per the BUILD file, this test is only run when CLP is disabled."""
        solver = ClpSolver()
        self.assertFalse(solver.available())
