import unittest
import warnings

import numpy as np

from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.csdp import CsdpSolver
import pydrake.solvers.sdpa_free_format as sdpa_free_format


class TestCsdpSolver(unittest.TestCase):
    def _make_prog(self):
        prog = mp.MathematicalProgram()
        x1 = prog.NewSymmetricContinuousVariables(2, "x1")
        x2 = prog.NewSymmetricContinuousVariables(3, "x2")
        y = prog.NewContinuousVariables(2, "y")
        prog.AddPositiveSemidefiniteConstraint(x1)
        prog.AddPositiveSemidefiniteConstraint(x2)
        prog.AddLinearConstraint(y[0] >= 0)
        prog.AddLinearConstraint(y[1] >= 0)
        prog.AddLinearEqualityConstraint(3*x1[0, 0] + 2*x1[0, 1] + 3*x1[1, 1]
                                         + y[0] == 1)
        prog.AddLinearEqualityConstraint(3*x2[0, 0] + 4*x2[1, 1] + 2*x2[0, 2]
                                         + 5*x2[2, 2] + y[1] == 2)
        prog.AddLinearCost(-(2*x1[0, 0] + 2*x1[0, 1] + 2*x1[1, 1] + 3*x2[0, 0]
                             + 2*x2[1, 1] + 2*x2[0, 2] + 3*x2[2, 2]))
        x1_expected = 0.125 * np.ones((2, 2))
        return prog, x1, x1_expected

    def test_csdp_solver(self):
        prog, x1, x1_expected = self._make_prog()
        solver = CsdpSolver()
        self.assertEqual(solver.solver_id(), CsdpSolver.id())
        self.assertTrue(solver.available())
        self.assertEqual(solver.solver_id().name(), "CSDP")
        self.assertEqual(solver.SolverName(), "CSDP")
        self.assertEqual(solver.solver_type(), mp.SolverType.kCsdp)
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        self.assertTrue(np.allclose(result.GetSolution(x1), x1_expected))
        self.assertEqual(result.get_solver_details().return_code, 0)
        np.testing.assert_allclose(
            result.get_solver_details().primal_objective, 2.75)
        np.testing.assert_allclose(
            result.get_solver_details().dual_objective, 2.75)
        np.testing.assert_allclose(
            result.get_solver_details().y_val, np.array([0.75, 1.]))
        z_expected = np.zeros((7, 7))
        z_expected[0, :2] = [0.25, -0.25]
        z_expected[1, :2] = [-0.25, 0.25]
        z_expected[3:, 3:] = np.diag([2., 2., 0.75, 1.])
        np.testing.assert_allclose(
            result.get_solver_details().Z_val.toarray(), z_expected, atol=1e-8)

        # Test removing free variables with a non-default method.
        solver = CsdpSolver(
            method=sdpa_free_format.RemoveFreeVariableMethod.kLorentzConeSlack)
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())

    def unavailable(self):
        """Per the BUILD file, this test is only run when CSDP is disabled."""
        solver = CsdpSolver()
        self.assertFalse(solver.available())
