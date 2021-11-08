import unittest

import numpy as np

from pydrake.common.test_utilities import numpy_compare
from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.ibex import IbexSolver


class TestIbexSolver(unittest.TestCase):
    def test_ibex_solver(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddBoundingBoxConstraint(-1., 1., x[0])
        prog.AddBoundingBoxConstraint(-1., 1., x[1])
        prog.AddCost(x[0] - x[1])
        solver = IbexSolver()
        self.assertEqual(solver.solver_id(), IbexSolver.id())

        self.assertTrue(solver.available())
        self.assertEqual(solver.solver_id().name(), "IBEX")
        self.assertEqual(solver.SolverName(), "IBEX")
        self.assertEqual(solver.solver_type(), mp.SolverType.kIbex)

        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        numpy_compare.assert_float_allclose(
            result.GetSolution(x), [-1.0, 1.0], atol=1E-7)

    def unavailable(self):
        """Per the BUILD file, this test is only run when Ibex is disabled."""
        solver = IbexSolver()
        self.assertFalse(solver.available())
