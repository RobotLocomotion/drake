import unittest
import warnings

import numpy as np

from pydrake.common.test_utilities import numpy_compare
from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.nlopt import NloptSolver


class TestNloptSolver(unittest.TestCase):
    def test_nlopt_solver(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint(x[0] >= 1)
        prog.AddLinearConstraint(x[1] >= 1)
        prog.AddQuadraticCost(np.eye(2), np.zeros(2), x)
        x_expected = np.array([1., 1.])

        solver = NloptSolver()
        self.assertEqual(solver.solver_id(), NloptSolver.id())
        self.assertTrue(solver.available())
        self.assertEqual(solver.solver_id().name(), "NLopt")
        self.assertEqual(solver.SolverName(), "NLopt")
        self.assertEqual(solver.solver_type(), mp.SolverType.kNlopt)
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        numpy_compare.assert_float_allclose(
            result.GetSolution(x), x_expected, atol=1E-7)
        self.assertEqual(result.get_solver_details().status, 4)

    def unavailable(self):
        """Per the BUILD file, this test is only run when NLopt is disabled."""
        solver = NloptSolver()
        self.assertFalse(solver.available())
