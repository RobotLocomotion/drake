from __future__ import absolute_import, division, print_function

import unittest

import numpy as np

from pydrake.common.test_utilities import numpy_compare
from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.scs import ScsSolver


class TestScsSolver(unittest.TestCase):
    def test_scs_solver(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint(x[0] >= 1)
        prog.AddLinearConstraint(x[1] >= 1)
        prog.AddQuadraticCost(np.eye(2), np.zeros(2), x)
        solver = ScsSolver()

        self.assertTrue(solver.available())
        self.assertEqual(solver.solver_id().name(), "SCS")
        self.assertEqual(solver.SolverName(), "SCS")
        self.assertEqual(solver.solver_type(), mp.SolverType.kScs)

        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        numpy_compare.assert_float_allclose(
            result.GetSolution(x), [1.0, 1.0], atol=1E-7)
        numpy_compare.assert_float_allclose(
            result.get_solver_details().primal_objective, 1.0, atol=1E-7)

    def unavailable(self):
        """Per the BUILD file, this test is only run when SCS is disabled."""
        solver = ScsSolver()
        self.assertFalse(solver.available())
