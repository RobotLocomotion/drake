import unittest

import numpy as np

from pydrake.solvers import (
    MakeSemidefiniteRelaxation,
    MathematicalProgram,
    SemidefiniteRelaxationOptions,
)
from pydrake.symbolic import Variables


class TestSemidefiniteRelaxation(unittest.TestCase):
    def test_MakeSemidefiniteRelaxation(self):
        prog = MathematicalProgram()
        y = prog.NewContinuousVariables(2, "y")
        A = np.array([[0.5, 0.7], [-0.2, 0.4], [-2.3, -4.5]])
        lb = np.array([1.3, -0.24, 0.25])
        ub = np.array([5.6, 0.1, 1.4])
        prog.AddLinearConstraint(A, lb, ub, y)
        options = SemidefiniteRelaxationOptions()
        relaxation = MakeSemidefiniteRelaxation(prog=prog, options=options)

        self.assertEqual(relaxation.num_vars(), 6)
        self.assertEqual(len(relaxation.positive_semidefinite_constraints()), 1)
        self.assertEqual(len(relaxation.linear_equality_constraints()), 1)
        self.assertEqual(len(relaxation.linear_constraints()), 2)

    def test_MakeSemidefiniteRelaxationWithGroups(self):
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        y = prog.NewContinuousVariables(3, "y")

        x_vars = Variables(x)
        y_vars = Variables(y)

        A = np.array([[0.5, 0.7], [-0.2, 0.4], [-2.3, -4.5]])
        lb = np.array([1.3, -0.24, 0.25])
        ub = np.array([5.6, 0.1, 1.4])
        prog.AddLinearConstraint(A, lb, ub, x)
        prog.AddQuadraticConstraint(A @ A.T, lb, 0, 5, y)
        options = SemidefiniteRelaxationOptions()
        relaxation = MakeSemidefiniteRelaxation(
            prog=prog, variable_groups=[x_vars, y_vars], options=options
        )

        # The semidefinite variables have 4 and 3 rows respectively,
        # with the "1" variable double counted. Therefore, there are
        # 5 choose 2 + 4 choose 2 - 1 variables.
        self.assertEqual(relaxation.num_vars(), 10 + 6 - 1)
        self.assertEqual(len(relaxation.positive_semidefinite_constraints()), 2)
        self.assertEqual(len(relaxation.linear_equality_constraints()), 1)
        self.assertEqual(len(relaxation.linear_constraints()), 2 + 1)

    def test_MakeSemidefiniteRelaxationOptions(self):
        options = SemidefiniteRelaxationOptions()
        options.add_implied_linear_equality_constraints = True
        options.add_implied_linear_constraints = True
        self.assertTrue(options.add_implied_linear_equality_constraints)
        self.assertTrue(options.add_implied_linear_constraints)

        options.set_to_weakest()
        self.assertFalse(options.add_implied_linear_equality_constraints)
        self.assertFalse(options.add_implied_linear_constraints)

        options.set_to_strongest()
        self.assertTrue(options.add_implied_linear_constraints)
