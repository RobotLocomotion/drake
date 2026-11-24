import gc
import unittest
import weakref

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
        solver.SetProjectionSolverInterface(None)
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

    def test_projection_solver_interface_lifetime(self):
        """The solver must keep its projection solver interface alive, even
        when the caller drops every other reference to it, and must release
        the interface when it is replaced or cleared.
        """
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint([1, 1], 1, 1, x)
        prog.AddBoundingBoxConstraint(0, 1, x[1])
        prog.AddLinearCost(x[0])

        def make_spy(solver):
            projection_solver = ClarabelSolver()
            spy = weakref.ref(projection_solver)
            solver.SetProjectionSolverInterface(projection_solver)
            return spy

        def collect():
            for _ in range(3):
                gc.collect()

        def check_solve(solver):
            result = solver.Solve(prog, None, None)
            self.assertTrue(result.is_success())
            numpy_compare.assert_float_allclose(
                result.GetSolution(x), [0.0, 1.0], atol=1e-7
            )

        solver = ProjectedGradientDescentSolver()
        spy = make_spy(solver)
        collect()
        self.assertIsNotNone(spy())
        check_solve(solver)

        # Replacing the interface releases the old one.
        replacement_spy = make_spy(solver)
        collect()
        self.assertIsNone(spy())
        self.assertIsNotNone(replacement_spy())
        check_solve(solver)

        # Clearing the interface releases it.
        solver.SetProjectionSolverInterface(None)
        collect()
        self.assertIsNone(replacement_spy())
        check_solve(solver)

        # The solver and its interface are freed together.
        solver_spy = weakref.ref(solver)
        spy = make_spy(solver)
        del solver
        collect()
        self.assertIsNone(solver_spy())
        self.assertIsNone(spy())
