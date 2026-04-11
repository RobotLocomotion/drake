import unittest

import numpy as np

from pydrake.solvers import (
    MathematicalProgram,
    MobyLcpSolver,
    SolverType,
)


class TestMobyLcpSolver(unittest.TestCase):
    def _make_prog(self):
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(3)
        prog.AddLinearComplementarityConstraint(
            M=np.eye(3), q=np.ones(3), vars=x
        )
        x_expected = np.zeros(3)
        return prog, x, x_expected

    def test_mobylcp_solver(self):
        prog, x, x_expected = self._make_prog()
        solver = MobyLcpSolver()
        self.assertEqual(solver.solver_id(), MobyLcpSolver.id())
        self.assertTrue(solver.available())
        self.assertEqual(solver.solver_id().name(), "Moby LCP")
        self.assertEqual(solver.SolverName(), "Moby LCP")
        self.assertEqual(solver.solver_type(), SolverType.kMobyLCP)
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        np.testing.assert_allclose(result.GetSolution(x), x_expected)
