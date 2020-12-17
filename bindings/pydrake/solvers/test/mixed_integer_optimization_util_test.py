import unittest
import numpy as np
from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers import mixed_integer_optimization_util as mip_util


class TestMixedIntegerOptimizationUtil(unittest.TestCase):
    def test_AddLogarithmicSos2Constraint(self):
        prog = mp.MathematicalProgram()
        lambdas = prog.NewContinuousVariables(3)
        # we use +lambdas to convert a numpy array of symbolic variables to a
        # numpy array of symbolic expressions
        y = mip_util.AddLogarithmicSos2Constraint(prog, +lambdas, "y")
        self.assertEqual(y.shape, (1,))

        def check_val(y_val, lambdas_val, satisfied_expected):
            x_val = np.zeros(prog.num_vars())
            prog.SetDecisionVariableValueInVector(lambdas, lambdas_val, x_val)
            prog.SetDecisionVariableValueInVector(y, y_val, x_val)
            satisfied = True
            for binding in prog.GetAllConstraints():
                satisfied = satisfied and binding.evaluator().CheckSatisfied(
                    prog.GetBindingVariableValues(binding, x_val))
            self.assertEqual(satisfied, satisfied_expected)

        check_val([0], [0.2, 0.8, 0.], True)
        check_val([0], [0.2, 0.7, 0.], False)
        check_val([1], [0.2, 0.7, 0.], False)
        check_val([1], [0., 0.3, 0.7], True)

    def test_AddSos2Constraint(self):
        prog = mp.MathematicalProgram()
        lambdas = prog.NewContinuousVariables(3)
        y = prog.NewBinaryVariables(2)
        # we use +lambdas to convert a numpy array of symbolic variables to a
        # numpy array of symbolic expressions
        mip_util.AddSos2Constraint(prog, +lambdas, +y)

        def check_val(y_val, lambdas_val, satisfied_expected):
            x_val = np.zeros(prog.num_vars())
            prog.SetDecisionVariableValueInVector(lambdas, lambdas_val, x_val)
            prog.SetDecisionVariableValueInVector(y, y_val, x_val)
            satisfied = True
            for binding in prog.GetAllConstraints():
                satisfied = satisfied and binding.evaluator().CheckSatisfied(
                    prog.GetBindingVariableValues(binding, x_val))
            self.assertEqual(satisfied, satisfied_expected)

        check_val([1, 0], [0.2, 0.8, 0.], True)
        check_val([1, 0], [0.2, 0.7, 0.], False)
        check_val([0, 1], [0.2, 0.7, 0.], False)
        check_val([0, 1], [0., 0.3, 0.7], True)
        check_val([1, 1], [0., 0.3, 0.7], False)
        check_val([0, 0], [0., 0.3, 0.7], False)

    def test_AddLogarithmicSos1Constraint(self):
        prog = mp.MathematicalProgram()
        (lambdas, y) = mip_util.AddLogarithmicSos1Constraint(prog, 4)

        def check_val(y_val, lambdas_val, satisfied_expected):
            x_val = np.zeros(prog.num_vars())
            prog.SetDecisionVariableValueInVector(lambdas, lambdas_val, x_val)
            prog.SetDecisionVariableValueInVector(y, y_val, x_val)
            satisfied = True
            for binding in prog.GetAllConstraints():
                satisfied = satisfied and binding.evaluator().CheckSatisfied(
                    prog.GetBindingVariableValues(binding, x_val))
            self.assertEqual(satisfied, satisfied_expected)

        check_val([1, 0], [0, 0, 0, 1], True)
        check_val([1, 0], [0, 0, 1, 0], False)
        check_val([1, 0], [0, 0, 0.5, 0.5], False)
