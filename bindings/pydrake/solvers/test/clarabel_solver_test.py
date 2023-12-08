import unittest

import numpy as np

from pydrake.solvers import (
    ClarabelSolver,
    ClarabelSolverDetails,
    MathematicalProgram,
)


class TestClarabelSolver(unittest.TestCase):

    def test_attributes(self):
        dut = ClarabelSolver()
        self.assertEqual(dut.solver_id(), ClarabelSolver.id())
        self.assertEqual(dut.solver_id().name(), "Clarabel")
        self.assertEqual(dut.SolverName(), "Clarabel")

    def test_solve(self):
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(1, "x")
        prog.AddLinearCost(x[0])
        prog.AddLinearConstraint(x[0] >= 1)
        result = ClarabelSolver().Solve(prog)
        details = result.get_solver_details()
        self.assertTrue(result.is_success())
        self.assertTrue(np.allclose(result.GetSolution(x), [1.0]))
        self.assertIsInstance(details, ClarabelSolverDetails)
        self.assertIsInstance(details.solve_time, float)
        self.assertIsInstance(details.iterations, int)
        self.assertEqual(details.status, "Solved")

    def unavailable(self):
        """Per the BUILD file, this test is only run when Clarabel is
        disabled."""
        solver = ClarabelSolver()
        self.assertFalse(solver.available())
