import unittest

import numpy as np

from pydrake.common.test_utilities import numpy_compare
from pydrake.solvers import (
    ClarabelSolver,
    MathematicalProgram,
    ProjectedGradientDescentSolver,
    SolutionResult,
    SolverType,
)


class TestProjectedGradientDescentSolver(unittest.TestCase):
    def test_projected_gradient_descent_solver(self):
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint([1, 1], 1, 1, x)
        prog.AddBoundingBoxConstraint(0, 1, x[1])
        prog.AddLinearCost(x[0])

        solver = ProjectedGradientDescentSolver()
        self.assertEqual(solver.solver_id(),
                         ProjectedGradientDescentSolver.id())
        self.assertTrue(solver.available())
        self.assertTrue(solver.enabled())

        # Test that the solver works.
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        numpy_compare.assert_float_allclose(
            result.GetSolution(x), [0., 1.], atol=1E-7)

        # Test that we can specify a projection solver interface.
        projection_solver = ClarabelSolver()
        solver.SetProjectionSolverInterface(
            projection_solver_interface=projection_solver)

        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        numpy_compare.assert_float_allclose(
            result.GetSolution(x), [0., 1.], atol=1E-7)

        # Test that we can specify a custom gradient function.
        def custom_gradient_function(y):
            return np.array([1.0, 0.0])
        solver.SetCustomGradientFunction(
            custom_gradient_function=custom_gradient_function)
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        numpy_compare.assert_float_allclose(
            result.GetSolution(x), [0., 1.], atol=1E-7)

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
            custom_projection_function=custom_projection_function)
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        numpy_compare.assert_float_allclose(
            result.GetSolution(x), [0., 1.], atol=1E-7)

        self.assertTrue(
            isinstance(
                ProjectedGradientDescentSolver.ConvergenceTolOptionName(),
                str))
        self.assertTrue(
            isinstance(
                ProjectedGradientDescentSolver.MaxIterationsOptionName(),
                str))
        self.assertTrue(
            isinstance(
                ProjectedGradientDescentSolver.BacktrackingCOptionName(),
                str))
        self.assertTrue(
            isinstance(
                ProjectedGradientDescentSolver.BacktrackingTauOptionName(),
                str))
        self.assertTrue(
            isinstance(
                ProjectedGradientDescentSolver.BacktrackingAlpha0OptionName(),
                str))

        self.assertTrue(
            isinstance(
                ProjectedGradientDescentSolver.kDefaultConvergenceTol,
                float))
        self.assertTrue(
            isinstance(
                ProjectedGradientDescentSolver.kDefaultMaxIterations,
                int))
        self.assertTrue(
            isinstance(
                ProjectedGradientDescentSolver.kDefaultBacktrackingC,
                float))
        self.assertTrue(
            isinstance(
                ProjectedGradientDescentSolver.kDefaultBacktrackingTau,
                float))
        self.assertTrue(
            isinstance(
                ProjectedGradientDescentSolver.kDefaultBacktrackingAlpha0,
                float))
        self.assertTrue(
            isinstance(
                ProjectedGradientDescentSolver.kDefaultMaxLineSearchSteps,
                int))
