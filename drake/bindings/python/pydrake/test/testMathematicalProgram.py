from __future__ import print_function, absolute_import

import unittest
import numpy as np
import pydrake
from pydrake.solvers import mathematicalprogram as mp
import pydrake.symbolic as sym


class TestMathematicalProgram(unittest.TestCase):
    def test_program_construction(self):
        prog = mp.MathematicalProgram()
        vars = prog.NewContinuousVariables(5, "x")
        self.assertEqual(vars.dtype, sym.Variable)

    def test_mixed_integer_optimization(self):
        prog = mp.MathematicalProgram()
        x = prog.NewBinaryVariables(3, "x")
        c = np.array([-1.0, -1.0, -2.0])
        prog.AddLinearCost(c.dot(x))
        a = np.array([1.0, 2.0, 3.0])
        prog.AddLinearConstraint(a.dot(x) <= 4)
        prog.AddLinearConstraint(x[0] + x[1], 1, np.inf)
        result = prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)

        # Test that we got the right solution for all x
        x_expected = np.array([1.0, 0.0, 1.0])
        self.assertTrue(np.all(np.isclose(prog.GetSolution(x), x_expected)))

        # Also test by asking for the value of each element of x
        for i in range(3):
            self.assertAlmostEqual(prog.GetSolution(x[i]), x_expected[i])

    def test_qp(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint(x[0] >= 1)
        prog.AddLinearConstraint(x[1] >= 1)
        prog.AddQuadraticCost(np.eye(2), np.zeros(2), x)
        result = prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)

        x_expected = np.array([1, 1])
        self.assertTrue(np.allclose(prog.GetSolution(x), x_expected))

    def test_symbolic_qp(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint(x[0] >= 1)
        prog.AddLinearConstraint(x[1] >= 1)
        prog.AddQuadraticCost(x[0]**2 + x[1]**2)
        result = prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)

        x_expected = np.array([1, 1])
        self.assertTrue(np.allclose(prog.GetSolution(x), x_expected))

    def test_bindings(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint(x[0] >= 1)
        prog.AddLinearConstraint(x[1] >= 1)
        prog.AddLinearConstraint(3 * x[0] - x[1] <= 2)
        prog.AddLinearConstraint(x[0] + 2 * x[1] == 3)
        cost = prog.AddQuadraticCost(0.5 * (x[0]**2 + x[1]**2))
        self.assertTrue(np.allclose(cost.constraint().Q(), np.eye(2)))
        self.assertTrue(np.allclose(cost.constraint().b(), np.zeros(2)))

        for (i, binding) in enumerate(prog.bounding_box_constraints()):
            constraint = binding.constraint()
            self.assertEqual(
                prog.FindDecisionVariableIndex(binding.variables()[0]),
                prog.FindDecisionVariableIndex(x[i]))
            self.assertTrue(np.allclose(constraint.A(), np.ones(1)))
            self.assertEqual(constraint.lower_bound(), 1)
            self.assertEqual(constraint.upper_bound(), np.inf)

        for (i, binding) in enumerate(prog.linear_constraints()):
            constraint = binding.constraint()
            self.assertEqual(
                prog.FindDecisionVariableIndex(binding.variables()[0]),
                prog.FindDecisionVariableIndex(x[0]))
            self.assertEqual(
                prog.FindDecisionVariableIndex(binding.variables()[1]),
                prog.FindDecisionVariableIndex(x[1]))
            self.assertTrue(np.allclose(constraint.A(), [-3, 1]))
            self.assertTrue(constraint.lower_bound(), -2)
            self.assertTrue(constraint.upper_bound(), np.inf)

        for (i, binding) in enumerate(prog.linear_equality_constraints()):
            constraint = binding.constraint()
            self.assertEqual(
                prog.FindDecisionVariableIndex(binding.variables()[0]),
                prog.FindDecisionVariableIndex(x[0]))
            self.assertEqual(
                prog.FindDecisionVariableIndex(binding.variables()[1]),
                prog.FindDecisionVariableIndex(x[1]))
            self.assertTrue(np.allclose(constraint.A(), [1, 2]))
            self.assertTrue(constraint.lower_bound(), 3)
            self.assertTrue(constraint.upper_bound(), 3)

        result = prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)

        x_expected = np.array([1, 1])
        self.assertTrue(np.allclose(prog.GetSolution(x), x_expected))

    def test_eval_binding(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        constraints = [prog.AddLinearConstraint(x[0] >= 1),
                       prog.AddLinearConstraint(x[1] >= 1)]
        cost = prog.AddLinearCost(np.sum(x))
        result = prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)
        x_expected = np.array([1, 1])
        self.assertTrue(np.allclose(prog.GetSolution(x), x_expected))
        for constraint in constraints:
            self.assertTrue(
                np.isclose(prog.EvalBindingAtSolution(constraint), 1))
        self.assertTrue(np.isclose(prog.EvalBindingAtSolution(cost), 2))

    def test_matrix_variables(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, 2, "x")
        for i in range(2):
            for j in range(2):
                prog.AddLinearConstraint(x[i, j] == 2 * i + j)
        prog.Solve()
        xval = prog.GetSolution(x)
        for i in range(2):
            for j in range(2):
                self.assertAlmostEqual(xval[i, j], 2 * i + j)
                self.assertEqual(xval[i, j], prog.GetSolution(x[i, j]))


if __name__ == '__main__':
    unittest.main()
