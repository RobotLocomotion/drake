import unittest

import numpy as np

from pydrake.autodiffutils import (
    AutoDiffXd,
    InitializeAutoDiff,
)
from pydrake.solvers import (
    AugmentedLagrangianNonsmooth,
    AugmentedLagrangianSmooth,
    MathematicalProgram,
)


class TestAugmentedLagrangian(unittest.TestCase):
    def setUp(self):
        self.prog = MathematicalProgram()
        x = self.prog.NewContinuousVariables(2)
        self.prog.AddQuadraticCost(x[0] * x[0] + 2 * x[1] * x[1])
        self.prog.AddLinearConstraint(x[0] + x[1] <= 3)

    def test_eval_double_nonsmooth(self):
        dut = AugmentedLagrangianNonsmooth(prog=self.prog,
                                           include_x_bounds=True)
        x_val = np.array([1., 3])
        lambda_val = np.array([0.5])
        al_val, constraint_residue, cost = dut.Eval(x=x_val,
                                                    lambda_val=lambda_val,
                                                    mu=0.1)
        self.assertIsInstance(al_val, float)
        self.assertIsInstance(constraint_residue, np.ndarray)
        self.assertIsInstance(cost, float)

    def test_eval_double_smooth(self):
        dut = AugmentedLagrangianSmooth(prog=self.prog,
                                        include_x_bounds=True)
        self.assertEqual(dut.s_size(), 1)
        x_val = np.array([1., 3])
        s_val = np.array([3.])
        lambda_val = np.array([0.5])
        al_val, constraint_residue, cost = dut.Eval(x=x_val,
                                                    s=s_val,
                                                    lambda_val=lambda_val,
                                                    mu=0.1)
        self.assertIsInstance(al_val, float)
        self.assertIsInstance(constraint_residue, np.ndarray)
        self.assertIsInstance(cost, float)

    def test_eval_ad_nonsmooth(self):
        dut = AugmentedLagrangianNonsmooth(prog=self.prog,
                                           include_x_bounds=True)
        x_val = InitializeAutoDiff(np.array([1., 3]))
        al_val, constraint_residue, cost = dut.Eval(x=x_val,
                                                    lambda_val=np.array([0.5]),
                                                    mu=0.1)
        self.assertIsInstance(al_val, AutoDiffXd)
        self.assertIsInstance(constraint_residue, np.ndarray)
        self.assertIsInstance(cost, AutoDiffXd)

    def test_eval_ad_smooth(self):
        dut = AugmentedLagrangianSmooth(prog=self.prog,
                                        include_x_bounds=True)
        x_val = InitializeAutoDiff(np.array([1., 3]))
        s_val = np.array([AutoDiffXd(1.)])
        al_val, constraint_residue, cost = dut.Eval(x=x_val,
                                                    s=s_val,
                                                    lambda_val=np.array([0.5]),
                                                    mu=0.1)
        self.assertIsInstance(al_val, AutoDiffXd)
        self.assertIsInstance(constraint_residue, np.ndarray)
        self.assertIsInstance(cost, AutoDiffXd)

    def test_lagrangian_size(self):
        self.assertEqual(
            AugmentedLagrangianNonsmooth(
                prog=self.prog, include_x_bounds=True).lagrangian_size(), 1)
        self.assertEqual(
            AugmentedLagrangianNonsmooth(
                prog=self.prog, include_x_bounds=False).lagrangian_size(), 1)
        self.assertEqual(
            AugmentedLagrangianSmooth(
                prog=self.prog, include_x_bounds=True).lagrangian_size(), 1)
        self.assertEqual(
            AugmentedLagrangianSmooth(
                prog=self.prog, include_x_bounds=False).lagrangian_size(), 1)

    def test_is_equality(self):
        self.assertEqual(
            AugmentedLagrangianNonsmooth(
                prog=self.prog, include_x_bounds=True).is_equality(), [False])
        self.assertEqual(
            AugmentedLagrangianSmooth(
                prog=self.prog, include_x_bounds=True).is_equality(), [False])
