import unittest
import numpy as np
from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers import mixed_integer_optimization_util as mip_util
from pydrake.solvers import branch_and_bound as bnb
from pydrake.solvers.clp import ClpSolver


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

    def test_AddBilinearProductMcCormickEnvelopeSos2(self):
        '''
            Test that this constraint works when using a linear binning option.
            The logarithmic binning case requires slightly more involved setup,
            but uses the same codepath, so it's not tested here.
        '''

        def setup_and_test_prog(
                setup_aux, expected_xyw, expected_Bx, expected_By):
            '''
            1) Setup an optimization with 1D continuous variables x, y, and
               w.
            2) Constraint x*y=w using this piecewise McCormick envelope
               constraint.
            3) Call `setup_aux(prog, x, y, w)`, which may mutate the program
               to add additional costs and constraints for testing.
            4) Solve the program and assert that the solver finds the
               expected xyw values and the expected binary setting.
            '''
            prog = mp.MathematicalProgram()

            w, x, y = prog.NewContinuousVariables(3)
            N_x_divisions = len(expected_Bx)
            N_y_divisions = len(expected_By)
            Bx = prog.NewBinaryVariables(N_x_divisions)
            By = prog.NewBinaryVariables(N_y_divisions)
            # Divide range [0, 1] into appropriate number of bins.
            phi_x = np.linspace(0., 1., N_x_divisions+1)
            phi_y = np.linspace(0., 1., N_y_divisions+1)
            binning = mip_util.IntervalBinning.kLinear
            mip_util.AddBilinearProductMcCormickEnvelopeSos2(
                prog=prog, x=x, y=y, w=w, phi_x=phi_x, phi_y=phi_y,
                Bx=Bx, By=By, binning=binning
            )
            setup_aux(prog, x, y, w)
            solver = bnb.MixedIntegerBranchAndBound(
                prog, ClpSolver().solver_id())
            solution_result = solver.Solve()
            self.assertEqual(solution_result, mp.SolutionResult.kSolutionFound)

            xyw = solver.GetSolution([x, y, w])
            self.assertTrue(np.allclose(xyw, expected_xyw))
            self.assertTrue(np.allclose(solver.GetSolution(Bx), expected_Bx))
            self.assertTrue(np.allclose(solver.GetSolution(By), expected_By))

        # Test some trivial costs that force xyw to their
        # extreme values.
        setup_and_test_prog(
            lambda prog, x, y, w: prog.AddLinearCost(x + y + w),
            expected_xyw=[0., 0., 0.],
            expected_Bx=[1.],
            expected_By=[1., 0., 0.]
        )
        setup_and_test_prog(
            lambda prog, x, y, w: prog.AddLinearCost(x - y - w),
            expected_xyw=[0., 1., 0.],
            expected_Bx=[1.],
            expected_By=[0., 0., 1.]
        )
        setup_and_test_prog(
            lambda prog, x, y, w: prog.AddLinearCost(-x - y - w),
            expected_xyw=[1., 1., 1.],
            expected_Bx=[1.],
            expected_By=[0., 0., 1.]
        )

        # Force x=1, w=0.5, (y=0.5), which should activate the
        # middle bin for y.
        def setup_aux(prog, x, y, w):
            prog.AddLinearEqualityConstraint(w == 0.5)
            prog.AddLinearEqualityConstraint(x == 1.)
        setup_and_test_prog(
            setup_aux,
            expected_xyw=[1., 0.5, 0.5],
            expected_Bx=[1.],
            expected_By=[0., 1., 0.]
        )
