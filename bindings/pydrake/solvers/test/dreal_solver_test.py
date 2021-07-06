import unittest
import warnings

import numpy as np

from pydrake.common.containers import EqualToDict
from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.dreal import DrealSolver
from pydrake.symbolic import Variable, logical_and, logical_or


class TestDrealSolver(unittest.TestCase):

    def test_dreal_solver(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(1, "x")
        prog.AddBoundingBoxConstraint(-1, 1, x)
        solver = DrealSolver()
        self.assertEqual(solver.solver_id(), DrealSolver.id())
        self.assertTrue(solver.available())
        self.assertEqual(solver.solver_id().name(), "dReal")
        self.assertEqual(solver.SolverName(), "dReal")
        self.assertEqual(solver.solver_type(), mp.SolverType.kDReal)
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())

    def test_interval(self):
        interval = DrealSolver.Interval(low=3.0, high=4.0)
        self.assertEqual(interval.low(), 3.0)
        self.assertEqual(interval.high(), 4.0)
        self.assertEqual(interval.diam(), 1.0)
        self.assertAlmostEqual(interval.mid(), 3.5, delta=1e-16)

    def test_dreal_satisfiability(self):
        x = Variable("x")
        f = logical_and(x > 1, x < 2)
        interval_box = EqualToDict(
            DrealSolver.CheckSatisfiability(f=f, delta=0.01))
        self.assertEqual(len(interval_box), 1)
        self.assertIsInstance(interval_box[x], DrealSolver.Interval)

    def test_local_optimization(self):
        self.assertNotEqual(
            DrealSolver.LocalOptimization.kUse,
            DrealSolver.LocalOptimization.kNotUse)

    def test_minimize(self):
        x = Variable("x")
        delta = 0.01
        interval_box = EqualToDict(
            DrealSolver.Minimize(
                objective=x**2,
                constraint=logical_and(x >= 1, x <= 10),
                delta=delta,
                local_optimization=DrealSolver.LocalOptimization.kUse))
        self.assertEqual(len(interval_box), 1)
        self.assertIsInstance(interval_box[x], DrealSolver.Interval)
        print(interval_box[x].diam())
        self.assertAlmostEqual(interval_box[x].mid(), 1.0, delta=delta)

    def unavailable(self):
        """Per the BUILD file, this test is only run when dReal is disabled."""
        solver = DrealSolver()
        self.assertFalse(solver.available())
