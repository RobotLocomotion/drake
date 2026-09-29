import unittest

import numpy as np

from pydrake.solvers import DaqpSolver, MathematicalProgram


class TestDaqpSolver(unittest.TestCase):
    def test_collision_qp(self):
        prog = MathematicalProgram()
        v = prog.NewContinuousVariables(2, "v")
        prog.AddQuadraticCost(2 * np.eye(2), np.array([16.0, -1.0]), v)
        collision = prog.AddLinearConstraint(v[0] >= -4.0)

        solver = DaqpSolver()
        self.assertEqual(solver.solver_id(), DaqpSolver.id())
        self.assertTrue(solver.available())
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())
        np.testing.assert_allclose(result.GetSolution(v), [-4.0, 0.5], atol=1e-7)
        np.testing.assert_allclose(result.GetDualSolution(collision), [8.0], atol=1e-7)
        self.assertEqual(result.get_solver_details().exitflag, 1)
