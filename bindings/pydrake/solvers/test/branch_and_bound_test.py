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

        prog.AddLinearConstraint(x[0] + 2 * x[1] + b[0] == 2.)
        prog.AddLinearConstraint(x[0] - 3.1 * b[1] >= 1)
        prog.AddLinearConstraint(b[1] + 1.2 * x[1] - b[0] <= 5)
        prog.AddQuadraticCost(x[0] * x[0])

        dut1 = MixedIntegerBranchAndBound(prog, OsqpSolver().solver_id())
        solution_result = dut1.Solve()
        self.assertEqual(solution_result, SolutionResult.kSolutionFound)
        self.assertAlmostEqual(dut1.GetOptimalCost(), 1.)
        self.assertAlmostEqual(dut1.GetSubOptimalCost(0), 1.)
        self.assertAlmostEqual(dut1.GetSolution(x[0], 0), 1.)
        self.assertAlmostEqual(dut1.GetSolution(x[0], 1), 1.)
        np.testing.assert_allclose(
            dut1.GetSolution(x, 0), [1., 0.], atol=1e-12)

        options = MixedIntegerBranchAndBound.Options()
        options.max_explored_nodes = 1
        dut2 = MixedIntegerBranchAndBound(
            prog=prog, solver_id=OsqpSolver().solver_id(), options=options)
        solution_result = dut2.Solve()
