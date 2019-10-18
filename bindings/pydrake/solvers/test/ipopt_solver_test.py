from __future__ import absolute_import, division, print_function

import unittest
import warnings

import numpy as np

from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.ipopt import IpoptSolver


class TestIpoptSolver(unittest.TestCase):
    def _make_prog(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint(x[0] >= 1)
        prog.AddLinearConstraint(x[1] >= 1)
        prog.AddQuadraticCost(np.eye(2), np.zeros(2), x)
        x_expected = np.array([1, 1])
        return prog, x, x_expected

    def test_ipopt_solver(self):
        prog, x, x_expected = self._make_prog()
        solver = IpoptSolver()
        self.assertTrue(solver.available())
        self.assertEqual(solver.solver_id().name(), "IPOPT")
        self.assertEqual(solver.SolverName(), "IPOPT")
        self.assertEqual(solver.solver_type(), mp.SolverType.kIpopt)
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        self.assertTrue(np.allclose(result.GetSolution(x), x_expected))
        self.assertEqual(result.get_solver_details().status, 0)
        self.assertEqual(result.get_solver_details().ConvertStatusToString(),
                         "Success")

    def unavailable(self):
        """Per the BUILD file, this test is only run when IPOPT is disabled."""
        solver = IpoptSolver()
        self.assertFalse(solver.available())
