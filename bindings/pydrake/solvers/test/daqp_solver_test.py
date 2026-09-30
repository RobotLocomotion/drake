import unittest

import numpy as np

from pydrake.solvers import (
    DaqpSolver,
    DaqpSolverDetails,
    MathematicalProgram,
    SolverOptions,
)


class TestDaqpSolver(unittest.TestCase):
    def _make_program(self):
        # The clipped collision example from Drake's differential IK system
        # test: desired velocity (-8, 0.5), with collision constraint
        # v_x >= -4.
        prog = MathematicalProgram()
        v = prog.NewContinuousVariables(2, "v")
        prog.AddQuadraticCost(2 * np.eye(2), np.array([16.0, -1.0]), v)
        collision = prog.AddLinearConstraint(v[0] >= -4.0)
        return prog, v, collision

    def test_attributes(self):
        dut = DaqpSolver()
        self.assertEqual(dut.solver_id(), DaqpSolver.id())
        self.assertEqual(dut.solver_id().name(), "DAQP")
        self.assertEqual(dut.SolverName(), "DAQP")
        self.assertTrue(dut.available())
        self.assertTrue(dut.enabled())

    def test_solve(self):
        prog, v, collision = self._make_program()
        result = DaqpSolver().Solve(prog)
        self.assertTrue(result.is_success())
        np.testing.assert_allclose(
            result.GetSolution(v), [-4.0, 0.5], atol=1e-10
        )
        np.testing.assert_allclose(
            result.GetDualSolution(collision), [8.0], atol=1e-10
        )
        details = result.get_solver_details()
        self.assertIsInstance(details, DaqpSolverDetails)
        self.assertEqual(details.exitflag, 1)
        self.assertIsInstance(details.iterations, int)
        self.assertIsInstance(details.setup_time, float)
        self.assertIsInstance(details.solve_time, float)
        # v_x >= -4 is parsed as a bounding box, so DAQP has only the two
        # variable bounds; its multiplier is negative at a lower bound.
        np.testing.assert_allclose(details.multipliers, [-8.0, 0.0], atol=1e-10)

    def test_options(self):
        prog, _, _ = self._make_program()
        options = SolverOptions()
        options.SetOption(DaqpSolver.id(), "primal_tol", 1e-10)
        options.SetOption(DaqpSolver.id(), "iter_limit", 100)
        result = DaqpSolver().Solve(prog, None, options)
        self.assertTrue(result.is_success())

    def unavailable(self):
        """Per the BUILD file, this test is only run when DAQP is
        disabled."""
        solver = DaqpSolver()
        self.assertFalse(solver.available())
