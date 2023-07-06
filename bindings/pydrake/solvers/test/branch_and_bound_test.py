import copy
import unittest

import numpy as np

from pydrake.solvers import (
    MathematicalProgram,
    MixedIntegerBranchAndBound,
    OsqpSolver,
    SolutionResult,
)


class TestMixedIntegerBranchAndBound(unittest.TestCase):
    def test(self):
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(2)
        b = prog.NewBinaryVariables(2)

        prog.AddLinearConstraint(b[0] + x[0] + 2 * x[1] == 2)
        prog.AddLinearConstraint(x[0] - 3.1 * b[1] >= 1)
        prog.AddLinearConstraint(b[1] + 1.2 * x[1] - b[0] <= 5)
        prog.AddQuadraticCost(x[0] * x[0])

        dut1 = MixedIntegerBranchAndBound(prog, OsqpSolver().solver_id())
        solution_result = dut1.Solve()
        self.assertEqual(solution_result, SolutionResult.kSolutionFound)

        # The solutions are either x = [1.0, 0.0] or x = [1.0, 0.5].
        # Both solutions have x₀ == 1.0 (and therefore cost == 1.0).
        self.assertAlmostEqual(dut1.GetOptimalCost(), 1.0)
        self.assertAlmostEqual(dut1.GetSubOptimalCost(0), 1.0)

        # Start with x₀ because its simpler (always 1.0).
        # Check the different overloads for GetSolution.
        self.assertAlmostEqual(dut1.GetSolution(x[0], 0), 1.0)
        self.assertAlmostEqual(dut1.GetSolution(x[0], 1), 1.0)
        self.assertAlmostEqual(dut1.GetSolution(x, 0)[0], 1.0)
        self.assertAlmostEqual(dut1.GetSolution(x, 1)[0], 1.0)
        # For x₁ the optimal vs suboptimal ordering is not deterministic, so we
        # need to allow for either order.
        x1_solutions = sorted([
            dut1.GetSolution(x[1], 0),
            dut1.GetSolution(x[1], 1),
        ])
        self.assertAlmostEqual(x1_solutions[0], 0.0)
        self.assertAlmostEqual(x1_solutions[1], 0.5)

        MixedIntegerBranchAndBound.Options()
        options = MixedIntegerBranchAndBound.Options(max_explored_nodes=22)
        options.max_explored_nodes = 1
        self.assertEqual(options.max_explored_nodes, 1)
        self.assertIn("max_explored_nodes=", repr(options))
        copy.copy(options)

        dut2 = MixedIntegerBranchAndBound(
            prog=prog, solver_id=OsqpSolver().solver_id(), options=options)
        solution_result = dut2.Solve()
