import unittest

import numpy as np

from pydrake.solvers import mathematicalprogram as mp

from pydrake.solvers import augmented_lagrangian as al
from pydrake.autodiffutils import InitializeAutoDiff


class TestAugmentedLagrangian(unittest.TestCase):
    def setUp(self):
        self.prog = mp.MathematicalProgram()
        x = self.prog.NewContinuousVariables(2)
        self.prog.AddQuadraticCost(x[0] * x[0] + 2 * x[1] * x[1])
        self.prog.AddLinearConstraint(x[0] + x[1] <= 3)

    def test_eval_double(self):
        x_val = np.array([1., 3])
        lambda_val = np.array([0.5])
        al_val, constraint_residue, cost = al.EvalAugmentedLagrangian(
            prog=self.prog,
            x=x_val,
            lambda_val=lambda_val,
            mu=0.1,
            include_x_bounds=True)

    def test_eval_ad(self):
        x_val = InitializeAutoDiff(np.array([1., 3]))
        al_val, constraint_residue, cost = al.EvalAugmentedLagrangian(
            prog=self.prog,
            x=x_val,
            lambda_val=np.array([0.5]),
            mu=0.1,
            include_x_bounds=True)

    def test_get_lagrangian_size_for_al(self):
        self.assertEqual(
            al.GetLagrangianSizeForAl(prog=self.prog, include_x_bounds=True),
            1)
        self.assertEqual(
            al.GetLagrangianSizeForAl(prog=self.prog, include_x_bounds=False),
            1)

    def test_get_constraint_type_for_al(self):
        self.assertEqual(
            al.IsEqualityForAl(prog=self.prog, include_x_bounds=True), [False])
