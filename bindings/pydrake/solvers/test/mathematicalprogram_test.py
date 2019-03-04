from __future__ import print_function, absolute_import

from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.gurobi import GurobiSolver
from pydrake.solvers.snopt import SnoptSolver
from pydrake.solvers.mathematicalprogram import (
    SolverOptions,
    SolverType
    )

import unittest
import warnings

import numpy as np

import pydrake
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.autodiffutils import AutoDiffXd
import pydrake.symbolic as sym

SNOPT_NO_GUROBI = SnoptSolver().available() and not GurobiSolver().available()


class TestQP:
    def __init__(self):
        # Create a simple QP that uses all deduced linear constraint types,
        # along with a quadratic and linear cost.
        # The solution should be [1, 1].
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        self.prog = prog
        self.x = x
        self.constraints = [
            # Bounding box
            prog.AddLinearConstraint(x[0] >= 1),
            # Bounding box
            prog.AddLinearConstraint(sym.logical_and(x[1] >= 1, x[1] <= 2.)),
            # Linear inequality
            prog.AddLinearConstraint(3 * x[0] - x[1] <= 2),
            # Linear equality
            prog.AddLinearConstraint(x[0] + 2 * x[1] == 3)]

        # TODO(eric.cousineau): Add constant terms
        self.costs = [prog.AddLinearCost(x[0] + x[1]),
                      prog.AddQuadraticCost(0.5 * (x[0]**2 + x[1]**2))]


class TestMathematicalProgram(unittest.TestCase):
    def test_program_construction(self):
        prog = mp.MathematicalProgram()
        vars = prog.NewContinuousVariables(5, "x")
        self.assertEqual(vars.dtype, sym.Variable)
        vars_all = prog.decision_variables()
        self.assertEqual(vars_all.shape, (5,))

    def test_program_attributes_and_solver_selection(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")

        # Add linear equality constraints; make sure the solver works.
        prog.AddLinearConstraint(x[0] + x[1] == 0)
        prog.AddLinearConstraint(2*x[0] - x[1] == 1)
        solver_id = mp.ChooseBestSolver(prog)
        self.assertEqual(solver_id.name(), "Linear system")
        solver = mp.MakeSolver(solver_id)
        self.assertEqual(solver.solver_id().name(), "Linear system")
        self.assertTrue(solver.AreProgramAttributesSatisfied(prog))
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())

        # With an inequality constraint added, the "Linear system" solver
        # doesn't work anymore.
        prog.AddLinearConstraint(x[0] >= 0)
        self.assertFalse(solver.AreProgramAttributesSatisfied(prog))
        with self.assertRaises(ValueError):
            solver.Solve(prog, None, None)

        # A different solver will work, though.  We re-use the result object
        # (as a mutable output argument), and make sure that it changes.
        solver_id = mp.ChooseBestSolver(prog)
        self.assertNotEqual(solver_id.name(), "Linear system")
        solver = mp.MakeSolver(solver_id)
        solver.Solve(prog, None, None, result)
        self.assertTrue(result.is_success())
        self.assertEqual(result.get_solver_id().name(), solver_id.name())

    def test_module_level_solve_function_and_result_accessors(self):
        qp = TestQP()
        x_expected = np.array([1, 1])
        result = mp.Solve(qp.prog)
        self.assertTrue(result.is_success())
        self.assertTrue(np.allclose(result.get_x_val(), x_expected))
        self.assertEqual(result.get_solution_result(),
                         mp.SolutionResult.kSolutionFound)
        self.assertEqual(result.get_optimal_cost(), 3.0)
        self.assertTrue(result.get_solver_id().name())
        self.assertTrue(np.allclose(result.GetSolution(), x_expected))
        self.assertEqual(result.GetSolution(qp.x[0]), 1.0)
        self.assertTrue(np.allclose(result.GetSolution(qp.x), x_expected))

    # TODO(jwnimmer-tri) MOSEK is also able to solve mixed integer programs;
    # perhaps we should test both of them?
    @unittest.skipUnless(GurobiSolver().available(), "Requires Gurobi")
    def test_mixed_integer_optimization(self):
        prog = mp.MathematicalProgram()
        x = prog.NewBinaryVariables(3, "x")
        c = np.array([-1.0, -1.0, -2.0])
        prog.AddLinearCost(c.dot(x))
        a = np.array([1.0, 2.0, 3.0])
        prog.AddLinearConstraint(a.dot(x) <= 4)
        prog.AddLinearConstraint(x[0] + x[1], 1, np.inf)
        solver = GurobiSolver()
        result = solver.Solve(prog, None, None)
        self.assertTrue(result.is_success())

        # Test that we got the right solution for all x
        x_expected = np.array([1.0, 0.0, 1.0])
        self.assertTrue(np.all(np.isclose(result.GetSolution(x), x_expected)))

        # Also test by asking for the value of each element of x
        for i in range(3):
            self.assertAlmostEqual(result.GetSolution(x[i]), x_expected[i])

    def test_qp(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearConstraint(x[0] >= 1)
        prog.AddLinearConstraint(x[1] >= 1)
        prog.AddQuadraticCost(np.eye(2), np.zeros(2), x)
        # Redundant cost just to check the spelling.
        prog.AddQuadraticErrorCost(vars=x, Q=np.eye(2),
                                   x_desired=np.zeros(2))
        prog.AddL2NormCost(A=np.eye(2), b=np.zeros(2), vars=x)

        result = mp.Solve(prog)
        self.assertTrue(result.is_success())

        x_expected = np.array([1, 1])
        self.assertTrue(np.allclose(result.GetSolution(x), x_expected))

    def test_symbolic_qp(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddConstraint(x[0], 1., 100.)
        prog.AddConstraint(x[1] >= 1)
        prog.AddQuadraticCost(x[0]**2 + x[1]**2)
        result = mp.Solve(prog)
        self.assertTrue(result.is_success())

        x_expected = np.array([1, 1])
        self.assertTrue(np.allclose(result.GetSolution(x), x_expected))

    def test_bindings(self):
        qp = TestQP()
        prog = qp.prog
        x = qp.x

        for binding in prog.GetAllCosts():
            self.assertIsInstance(binding.evaluator(), mp.Cost)
        for binding in prog.GetLinearConstraints():
            self.assertIsInstance(binding.evaluator(), mp.Constraint)
        for binding in prog.GetAllConstraints():
            self.assertIsInstance(binding.evaluator(), mp.Constraint)

        self.assertTrue(prog.linear_costs())
        for (i, binding) in enumerate(prog.linear_costs()):
            cost = binding.evaluator()
            self.assertTrue(np.allclose(cost.a(), np.ones((1, 2))))

        self.assertTrue(prog.quadratic_costs())
        for (i, binding) in enumerate(prog.quadratic_costs()):
            cost = binding.evaluator()
            self.assertTrue(np.allclose(cost.Q(), np.eye(2)))
            self.assertTrue(np.allclose(cost.b(), np.zeros(2)))

        self.assertTrue(prog.bounding_box_constraints())
        for (i, binding) in enumerate(prog.bounding_box_constraints()):
            constraint = binding.evaluator()
            self.assertEqual(
                prog.FindDecisionVariableIndex(binding.variables()[0]),
                prog.FindDecisionVariableIndex(x[i]))
            num_constraints = constraint.num_constraints()
            if num_constraints == 1:
                self.assertEqual(constraint.A(), 1)
                self.assertEqual(constraint.lower_bound(), 1)
                self.assertEqual(constraint.upper_bound(), np.inf)
            else:
                self.assertTrue(np.allclose(constraint.A(), np.eye(2)))
                self.assertTrue(np.allclose(constraint.lower_bound(),
                                            [1, -np.inf]))
                self.assertTrue(np.allclose(constraint.upper_bound(),
                                            [np.inf, 2]))

        self.assertTrue(prog.linear_constraints())
        for (i, binding) in enumerate(prog.linear_constraints()):
            constraint = binding.evaluator()
            self.assertEqual(
                prog.FindDecisionVariableIndex(binding.variables()[0]),
                prog.FindDecisionVariableIndex(x[0]))
            self.assertEqual(
                prog.FindDecisionVariableIndex(binding.variables()[1]),
                prog.FindDecisionVariableIndex(x[1]))
            self.assertTrue(np.allclose(constraint.A(), [3, -1]))
            self.assertTrue(constraint.lower_bound(), -2)
            self.assertTrue(constraint.upper_bound(), np.inf)

        self.assertTrue(prog.linear_equality_constraints())
        for (i, binding) in enumerate(prog.linear_equality_constraints()):
            constraint = binding.evaluator()
            self.assertEqual(
                prog.FindDecisionVariableIndex(binding.variables()[0]),
                prog.FindDecisionVariableIndex(x[0]))
            self.assertEqual(
                prog.FindDecisionVariableIndex(binding.variables()[1]),
                prog.FindDecisionVariableIndex(x[1]))
            self.assertTrue(np.allclose(constraint.A(), [1, 2]))
            self.assertTrue(constraint.lower_bound(), 3)
            self.assertTrue(constraint.upper_bound(), 3)

        result = mp.Solve(prog)
        self.assertTrue(result.is_success())

        x_expected = np.array([1, 1])
        self.assertTrue(np.allclose(result.GetSolution(x), x_expected))

        # Test deprecated method.
        with catch_drake_warnings(expected_count=1):
            c = binding.constraint()

    def test_constraint_api(self):
        prog = mp.MathematicalProgram()
        x0, = prog.NewContinuousVariables(1, "x")
        c = prog.AddLinearConstraint(x0 >= 2).evaluator()
        ce = prog.AddLinearEqualityConstraint(2*x0, 1).evaluator()

        self.assertTrue(c.CheckSatisfied([2.], tol=1e-3))
        self.assertFalse(c.CheckSatisfied([AutoDiffXd(1.)]))
        self.assertIsInstance(c.CheckSatisfied([x0]), sym.Formula)

        def check_bounds(c, A, lb, ub):
            self.assertTrue(np.allclose(c.A(), A))
            self.assertTrue(np.allclose(c.lower_bound(), lb))
            self.assertTrue(np.allclose(c.upper_bound(), ub))

        check_bounds(c, [1.], [2.], [np.inf])
        c.UpdateLowerBound([3.])
        check_bounds(c, [1.], [3.], [np.inf])
        c.UpdateUpperBound([4.])
        check_bounds(c, [1.], [3.], [4.])
        c.set_bounds([-10.], [10.])
        check_bounds(c, [1.], [-10.], [10.])
        c.UpdateCoefficients([10.], [-20.], [-30.])
        check_bounds(c, [10.], [-20.], [-30.])

        check_bounds(ce, [2.], [1.], [1.])
        ce.UpdateCoefficients([10.], [20.])
        check_bounds(ce, [10.], [20.], [20.])

    def test_cost_api(self):
        prog = mp.MathematicalProgram()
        x0, = prog.NewContinuousVariables(1, "x")
        lc = prog.AddLinearCost(1*x0 + 2).evaluator()
        qc = prog.AddQuadraticCost(0.5*x0**2 + 2*x0 + 3).evaluator()

        def check_linear_cost(cost, a, b):
            self.assertTrue(np.allclose(cost.a(), a))
            self.assertTrue(np.allclose(cost.b(), b))

        check_linear_cost(lc, [1.], 2.)
        lc.UpdateCoefficients([10.])
        check_linear_cost(lc, [10.], 0.)

        def check_quadratic_cost(cost, Q, b, c):
            self.assertTrue(np.allclose(cost.Q(), Q))
            self.assertTrue(np.allclose(cost.b(), b))
            self.assertTrue(np.allclose(cost.c(), c))

        check_quadratic_cost(qc, [1.], [2.], 3.)
        qc.UpdateCoefficients([10.], [20.])
        check_quadratic_cost(qc, [10.], [20.], 0)

    def test_eval_binding(self):
        qp = TestQP()
        prog = qp.prog

        x = qp.x
        x_expected = np.array([1., 1.])

        costs = qp.costs
        cost_values_expected = [2., 1.]
        constraints = qp.constraints
        constraint_values_expected = [1., 1., 2., 3.]

        with catch_drake_warnings(action='ignore'):
            prog.Solve()
            self.assertTrue(np.allclose(prog.GetSolution(x), x_expected))

            enum = zip(constraints, constraint_values_expected)
            for (constraint, value_expected) in enum:
                value = prog.EvalBindingAtSolution(constraint)
                self.assertTrue(np.allclose(value, value_expected))

            enum = zip(costs, cost_values_expected)
            for (cost, value_expected) in enum:
                value = prog.EvalBindingAtSolution(cost)
                self.assertTrue(np.allclose(value, value_expected))

            # Existence check.
            self.assertIsInstance(
                prog.EvalBinding(costs[0], x_expected), np.ndarray)
            self.assertIsInstance(
                prog.EvalBindings(prog.GetAllConstraints(), x_expected),
                np.ndarray)

            # Bindings for `Eval`.
            x_list = (float(1.), AutoDiffXd(1.), sym.Variable("x"))
            T_y_list = (float, AutoDiffXd, sym.Expression)
            evaluator = costs[0].evaluator()
            for x_i, T_y_i in zip(x_list, T_y_list):
                y_i = evaluator.Eval(x=[x_i, x_i])
                self.assertIsInstance(y_i[0], T_y_i)

    def test_matrix_variables(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, 2, "x")
        for i in range(2):
            for j in range(2):
                prog.AddLinearConstraint(x[i, j] == 2 * i + j)
        result = mp.Solve(prog)
        xval = result.GetSolution(x)
        for i in range(2):
            for j in range(2):
                self.assertAlmostEqual(xval[i, j], 2 * i + j)
                self.assertEqual(xval[i, j], result.GetSolution(x[i, j]))
        # Just check spelling.
        y = prog.NewIndeterminates(2, 2, "y")

    def test_sdp(self):
        prog = mp.MathematicalProgram()
        S = prog.NewSymmetricContinuousVariables(3, "S")
        prog.AddLinearConstraint(S[0, 1] >= 1)
        prog.AddPositiveSemidefiniteConstraint(S)
        prog.AddPositiveSemidefiniteConstraint(S+S)
        prog.AddLinearCost(np.trace(S))
        result = mp.Solve(prog)
        self.assertTrue(result.is_success())
        S = result.GetSolution(S)
        eigs = np.linalg.eigvals(S)
        tol = 1e-8
        self.assertTrue(np.all(eigs >= -tol))
        self.assertTrue(S[0, 1] >= -tol)

    def test_sos(self):
        # Find a,b,c,d subject to
        # a(0) + a(1)*x,
        # b(0) + 2*b(1)*x + b(2)*x^2 is SOS,
        # c(0)*x^2 + 2*c(1)*x*y + c(2)*y^2 is SOS,
        # d(0)*x^2 is SOS.
        # d(1)*x^2 is SOS.
        prog = mp.MathematicalProgram()
        x = prog.NewIndeterminates(1, "x")
        poly = prog.NewFreePolynomial(sym.Variables(x), 1)
        (poly, binding) = prog.NewSosPolynomial(sym.Variables(x), 2)
        y = prog.NewIndeterminates(1, "y")
        (poly, binding) = prog.NewSosPolynomial((sym.Monomial(x[0]),
                                                 sym.Monomial(y[0])))
        d = prog.NewContinuousVariables(2, "d")
        prog.AddSosConstraint(d[0]*x.dot(x))
        prog.AddSosConstraint(d[1]*x.dot(x), [sym.Monomial(x[0])])
        result = mp.Solve(prog)
        self.assertTrue(result.is_success())

        # Test SubstituteSolution(sym.Expression)
        with catch_drake_warnings(action='ignore'):
            prog.Solve()
            # TODO(eric.cousineau): Expose `SymbolicTestCase` so that other
            # tests can use the assertion utilities.
            self.assertEqual(
                prog.SubstituteSolution(d[0] + d[1]).Evaluate(),
                prog.GetSolution(d[0]) + prog.GetSolution(d[1]))
            # Test SubstituteSolution(sym.Polynomial)
            poly = d[0]*x.dot(x)
            poly_sub_actual = prog.SubstituteSolution(
                sym.Polynomial(poly, sym.Variables(x)))
            poly_sub_expected = sym.Polynomial(
                prog.SubstituteSolution(d[0])*x.dot(x), sym.Variables(x))
            # TODO(soonho): At present, these must be converted to `Expression`
            # to compare, because as `Polynomial`s the comparison fails with
            # `0*x(0)^2` != `0`, which indicates that simplification is not
            # happening somewhere.
            self.assertTrue(
                poly_sub_actual.ToExpression().EqualTo(
                    poly_sub_expected.ToExpression()),
                "{} != {}".format(poly_sub_actual, poly_sub_expected))

    def test_lcp(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, 'x')
        M = np.array([[1, 3], [4, 1]])
        q = np.array([-16, -15])
        binding = prog.AddLinearComplementarityConstraint(M, q, x)
        result = mp.Solve(prog)
        self.assertTrue(result.is_success())
        self.assertIsInstance(binding.evaluator(),
                              mp.LinearComplementarityConstraint)

    def test_linear_constraints(self):
        # TODO(eric.cousineau): Add more general tests
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, 'x')
        lb = [0., 0.]
        ub = [1., 1.]
        prog.AddBoundingBoxConstraint(lb, ub, x)
        prog.AddBoundingBoxConstraint(0., 1., x[0])
        prog.AddBoundingBoxConstraint(0., 1., x)
        prog.AddLinearConstraint(A=np.eye(2), lb=np.zeros(2), ub=np.ones(2),
                                 vars=x)
        prog.AddLinearConstraint(e=x[0], lb=0, ub=1)
        prog.AddLinearConstraint(v=x, lb=[0, 0], ub=[1, 1])
        prog.AddLinearConstraint(f=(x[0] == 0))

        prog.AddLinearEqualityConstraint(np.eye(2), np.zeros(2), x)
        prog.AddLinearEqualityConstraint(x[0] == 1)
        prog.AddLinearEqualityConstraint(x[0] + x[1], 1)

    def test_pycost_and_pyconstraint(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(1, 'x')

        def cost(x):
            return (x[0]-1.)*(x[0]-1.)

        def constraint(x):
            return x

        prog.AddCost(cost, vars=x)
        prog.AddConstraint(constraint, lb=[0.], ub=[2.], vars=x)
        result = mp.Solve(prog)
        self.assertAlmostEqual(result.GetSolution(x)[0], 1.)

    def test_addcost_symbolic(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(1, 'x')
        prog.AddCost((x[0]-1.)**2)
        prog.AddConstraint(0 <= x[0])
        prog.AddConstraint(x[0] <= 2)
        result = mp.Solve(prog)
        self.assertAlmostEqual(result.GetSolution(x)[0], 1.)

    def test_initial_guess(self):
        prog = mp.MathematicalProgram()
        count = 6
        shape = (2, 3)
        x = prog.NewContinuousVariables(count, 'x')
        x_matrix = x.reshape(shape)
        x0 = np.arange(count)
        x0_matrix = x0.reshape(shape)
        all_nan = np.full(x.shape, np.nan)
        self.assertTrue(np.isnan(prog.GetInitialGuess(x)).all())

        def check_and_reset():
            self.assertTrue((prog.GetInitialGuess(x) == x0).all())
            self.assertTrue(
                (prog.GetInitialGuess(x_matrix) == x0_matrix).all())
            prog.SetInitialGuess(x, all_nan)
            self.assertTrue(np.isnan(prog.GetInitialGuess(x)).all())

        # Test setting individual variables
        for i in range(count):
            prog.SetInitialGuess(x[i], x0[i])
            self.assertEqual(prog.GetInitialGuess(x[i]), x0[i])
        check_and_reset()

        # Test setting matrix values using both
        # 1d and 2d np arrays.
        prog.SetInitialGuess(x, x0)
        check_and_reset()
        prog.SetInitialGuess(x_matrix, x0_matrix)
        check_and_reset()

        # Test setting all values at once.
        prog.SetInitialGuessForAllVariables(x0)
        check_and_reset()

        # Check an extrinsic guess.  We sanity check changes to the guess using
        # loose "any" and "all" predicates rather than specific indices because
        # we should not presume how variables map into indices.
        guess = np.ndarray(count)
        guess.fill(np.nan)
        self.assertTrue(all([np.isnan(i) for i in guess]))
        prog.SetDecisionVariableValueInVector(x[0], x0[0], guess)
        self.assertFalse(all([np.isnan(i) for i in guess]))
        self.assertTrue(any([np.isnan(i) for i in guess]))
        prog.SetDecisionVariableValueInVector(x_matrix, x0_matrix, guess)
        self.assertFalse(any([np.isnan(i) for i in guess]))

    @unittest.skipIf(
        SNOPT_NO_GUROBI,
        "SNOPT is unable to solve this problem (#10653).")
    def test_lorentz_cone_constraint(self):
        # Set Up Mathematical Program
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        z = prog.NewContinuousVariables(1, "z")
        prog.AddCost(z[0])

        # Add LorentzConeConstraints
        prog.AddLorentzConeConstraint(np.array([0*x[0]+1, x[0]-1, x[1]-1]))
        prog.AddLorentzConeConstraint(np.array([z[0], x[0], x[1]]))

        # Test result
        result = mp.Solve(prog)
        self.assertTrue(result.is_success())

        # Check answer
        x_expected = np.array([1-2**(-0.5), 1-2**(-0.5)])
        self.assertTrue(np.allclose(result.GetSolution(x), x_expected))

    def test_solver_options(self):
        prog = mp.MathematicalProgram()

        prog.SetSolverOption(SolverType.kGurobi, "double_key", 1.0)
        prog.SetSolverOption(SolverType.kGurobi, "int_key", 2)
        prog.SetSolverOption(SolverType.kGurobi, "string_key", "3")

        options = prog.GetSolverOptions(SolverType.kGurobi)
        self.assertDictEqual(
            options, {"double_key": 1.0, "int_key": 2, "string_key": "3"})

        # For now, just make sure the constructor exists.  Once we bind more
        # accessors, we can test them here.
        options_object = SolverOptions()
