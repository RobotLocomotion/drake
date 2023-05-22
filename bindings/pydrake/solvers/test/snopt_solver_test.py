import unittest

import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.common.test_utilities import numpy_compare
from pydrake.solvers import (
    MathematicalProgram,
    SnoptSolver,
    SolutionResult,
    SolverType,
)


class TestSnoptSolver(unittest.TestCase):
    def test_snopt_solver(self):
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint(x[0] + x[1] == 1)
        prog.AddBoundingBoxConstraint(0, 1, x[1])
        prog.AddLinearCost(x[0])

        solver = SnoptSolver()
        self.assertEqual(solver.solver_id(), SnoptSolver.id())
        if solver.available():
            self.assertTrue(solver.enabled())
            self.assertEqual(solver.solver_type(), SolverType.kSnopt)

            result = solver.Solve(prog, None, None)
            self.assertTrue(result.is_success())
            numpy_compare.assert_float_allclose(
                result.GetSolution(x), [0., 1.], atol=1E-7)
            self.assertEqual(result.get_solver_details().info, 1)
            np.testing.assert_allclose(
                result.get_solver_details().xmul, np.array([0., -1]))
            np.testing.assert_allclose(
                result.get_solver_details().F, np.array([0, 1.]))
            np.testing.assert_allclose(
                result.get_solver_details().Fmul, np.array([0, 1.]))

    def test_solver_specific_error(self):
        # Intentionally write a constraint with incorrect gradients.
        def my_constraint(x):
            return [AutoDiffXd(np.sin(x[0].value()), np.ones(1))]
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(1, "x")
        prog.AddConstraint(my_constraint, [1], [1], x)
        solver = SnoptSolver()
        result = solver.Solve(prog)
        self.assertEqual(result.is_success(), False)
        self.assertEqual(result.get_solution_result(),
                         SolutionResult.kSolverSpecificError)
        self.assertEqual(result.get_solver_details().info, 41)
