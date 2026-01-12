import unittest

import numpy as np

from pydrake.common.test_utilities import numpy_compare
from pydrake.solvers import (
    ClarabelSolver,
    MathematicalProgram,
    ProjectedGradientDescentSolver,
)


class TestProjectedGradientDescentSolver(unittest.TestCase):
    def test_projected_gradient_descent_solver(self):
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint([1, 1], 1, 1, x)
        prog.AddBoundingBoxConstraint(0, 1, x[1])
        prog.AddLinearCost(x[0])

        solver = ProjectedGradientDescentSolver()
        PGDS = ProjectedGradientDescentSolver
        self.assertEqual(solver.solver_id(), PGDS.id())
        self.assertTrue(solver.available())
        self.assertTrue(solver.enabled())

        # Test that the solver works.
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        numpy_compare.assert_float_allclose(
            result.GetSolution(x), [0.0, 1.0], atol=1e-7
        )

        # Test that we can specify a projection solver interface.
        projection_solver = ClarabelSolver()
        solver.SetProjectionSolverInterface(
            projection_solver_interface=projection_solver
        )

        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        numpy_compare.assert_float_allclose(
            result.GetSolution(x), [0.0, 1.0], atol=1e-7
        )

        # Test that we can specify a custom gradient function.
        def custom_gradient_function(y):
            return np.array([1.0, 0.0])

        solver.SetCustomGradientFunction(
            custom_gradient_function=custom_gradient_function
        )
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        numpy_compare.assert_float_allclose(
            result.GetSolution(x), [0.0, 1.0], atol=1e-7
        )

        # Test that we can specify a custom projection function.
        def custom_projection_function(y):
            prog = MathematicalProgram()
            z = prog.NewContinuousVariables(2)
            prog.AddLinearConstraint([1, 1], 1, 1, z)
            prog.AddBoundingBoxConstraint(0, 1, z[1])
            prog.AddQuadraticErrorCost(1, y, z)
            result = projection_solver.Solve(prog)
            return result.is_success(), result.GetSolution(z)

        solver.SetCustomProjectionFunction(
            custom_projection_function=custom_projection_function
        )
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        numpy_compare.assert_float_allclose(
            result.GetSolution(x), [0.0, 1.0], atol=1e-7
        )

        self.assertIsInstance(PGDS.ConvergenceTolOptionName(), str)
        self.assertIsInstance(PGDS.MaxIterationsOptionName(), str)
        self.assertIsInstance(PGDS.BacktrackingCOptionName(), str)
        self.assertIsInstance(PGDS.BacktrackingTauOptionName(), str)
        self.assertIsInstance(PGDS.BacktrackingAlpha0OptionName(), str)

        self.assertIsInstance(PGDS.kDefaultConvergenceTol, float)
        self.assertIsInstance(PGDS.kDefaultMaxIterations, int)
        self.assertIsInstance(PGDS.kDefaultBacktrackingC, float)
        self.assertIsInstance(PGDS.kDefaultBacktrackingTau, float)
        self.assertIsInstance(PGDS.kDefaultBacktrackingAlpha0, float)
        self.assertIsInstance(PGDS.kDefaultMaxLineSearchSteps, int)
