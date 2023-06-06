import unittest
import warnings

import numpy as np

from pydrake.solvers import (
    UnrevisedLemkeSolver,
    MathematicalProgram,
    SolverType,
)


class TestUnrevisedLemkeSolver(unittest.TestCase):

    def _make_prog(self):
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(3)
        prog.AddLinearComplementarityConstraint(
            M=np.eye(3), q=np.ones(3), vars=x)
        x_expected = np.zeros(3)
        return prog, x, x_expected

    def test_unrevised_lemke_solver(self):
        prog, x, x_expected = self._make_prog()
        solver = UnrevisedLemkeSolver()
        self.assertEqual(solver.solver_id(), UnrevisedLemkeSolver.id())
        self.assertTrue(solver.available())
        self.assertEqual(solver.solver_id().name(), "Unrevised Lemke")
        self.assertEqual(solver.SolverName(), "Unrevised Lemke")
        self.assertEqual(solver.solver_type(), SolverType.kUnrevisedLemke)
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        np.testing.assert_allclose(result.GetSolution(x), x_expected)
