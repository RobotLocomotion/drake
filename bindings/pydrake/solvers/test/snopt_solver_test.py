from __future__ import absolute_import, division, print_function

import unittest

import numpy as np

from pydrake.common.test_utilities import numpy_compare
from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.snopt import SnoptSolver


class TestSnoptSolver(unittest.TestCase):
    def test_snopt_solver(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint(x[0] + x[1] == 1)
        prog.AddBoundingBoxConstraint(0, 1, x[1])
        prog.AddLinearCost(x[0])

        solver = SnoptSolver()
        if solver.available():
            self.assertEqual(solver.solver_type(), mp.SolverType.kSnopt)

            result = solver.Solve(prog, None, None)
            self.assertTrue(result.is_success())
            numpy_compare.assert_float_allclose(
                result.GetSolution(x), [0., 1.], atol=1E-7)
            self.assertEqual(result.get_solver_details().info, 1)
