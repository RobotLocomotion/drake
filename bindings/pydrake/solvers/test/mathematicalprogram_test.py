from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.gurobi import GurobiSolver
from pydrake.solvers.snopt import SnoptSolver
from pydrake.solvers.mathematicalprogram import (
    LinearConstraint,
    MathematicalProgramResult,
    SolverOptions,
    SolverType,
    SolverId,
    SolverInterface
    )

from functools import partial
import unittest
import warnings

import numpy as np

import pydrake
from pydrake.common import kDrakeAssertIsArmed
from pydrake.autodiffutils import AutoDiffXd
from pydrake.common.test_utilities import numpy_compare
from pydrake.forwarddiff import jacobian
from pydrake.math import ge
import pydrake.symbolic as sym


SNOPT_NO_GUROBI = SnoptSolver().available() and not GurobiSolver().available()
# MathematicalProgram is only bound for float and AutoDiffXd.
SCALAR_TYPES = [float, AutoDiffXd]


class TestCost(unittest.TestCase):
    def test_linear_cost(self):
        a = np.array([1., 2.])
        b = 0.5
        cost = mp.LinearCost(a, b)
        np.testing.assert_allclose(cost.a(), a)
        self.assertEqual(cost.b(), b)

    def test_quadratic_cost(self):
        Q = np.array([[1., 2.], [2., 3.]])
        b = np.array([3., 4.])
        c = 0.4
        cost = mp.QuadraticCost(Q, b, c)
        np.testing.assert_allclose(cost.Q(), Q)
        np.testing.assert_allclose(cost.b(), b)
        self.assertEqual(cost.c(), c)


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
        self.assertAlmostEqual(result.GetSolution(qp.x[0]), 1.0)
        self.assertTrue(np.allclose(result.GetSolution(qp.x), x_expected))
        self.assertTrue(result.GetSolution(sym.Expression(qp.x[0])).EqualTo(
            result.GetSolution(qp.x[0])))
        m = np.array([sym.Expression(qp.x[0]), sym.Expression(qp.x[1])])
        self.assertTrue(result.GetSolution(m)[1, 0].EqualTo(
            result.GetSolution(qp.x[1])))

        x_val_new = np.array([1, 2])
        result.set_x_val(x_val_new)
        np.testing.assert_array_equal(x_val_new, result.get_x_val())

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
        prog.AddConstraint(
            LinearConstraint(np.array([[1., 1.]]), np.array([1]),
                             np.array([np.inf])), [x[0], x[1]])
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
        # N.B. Scalar-wise logical ops work for Expression, but array ops need
        # the workaround overloads from `pydrake.math`.
        prog.AddLinearConstraint(ge(x, 1))
        prog.AddQuadraticCost(np.eye(2), np.zeros(2), x)
        prog.AddQuadraticCost(np.eye(2), np.zeros(2), 1, x)
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
        self.assertEqual(
            str(qp.constraints[0]),
            "BoundingBoxConstraint\n1 <= x(0) <= inf\n")
        prog = qp.prog
        x = qp.x

        self.assertEqual(prog.FindDecisionVariableIndices(vars=[x[0], x[1]]),
                         [0, 1])
        self.assertEqual(prog.decision_variable_index()[x[0].get_id()], 0)
        self.assertEqual(prog.decision_variable_index()[x[1].get_id()], 1)

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
            self.assertIsNone(cost.gradient_sparsity_pattern())

        self.assertTrue(prog.quadratic_costs())
        for (i, binding) in enumerate(prog.quadratic_costs()):
            cost = binding.evaluator()
            self.assertTrue(np.allclose(cost.Q(), np.eye(2)))
            self.assertTrue(np.allclose(cost.b(), np.zeros(2)))
            self.assertIsNone(cost.gradient_sparsity_pattern())

        self.assertTrue(prog.bounding_box_constraints())
        for (i, binding) in enumerate(prog.bounding_box_constraints()):
            constraint = binding.evaluator()
            self.assertEqual(
                prog.FindDecisionVariableIndex(var=binding.variables()[0]),
                prog.FindDecisionVariableIndex(var=x[i]))
            self.assertIsNone(constraint.gradient_sparsity_pattern())
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
            self.assertIsNone(constraint.gradient_sparsity_pattern())
            self.assertEqual(
                prog.FindDecisionVariableIndex(var=binding.variables()[0]),
                prog.FindDecisionVariableIndex(var=x[0]))
            self.assertEqual(
                prog.FindDecisionVariableIndex(var=binding.variables()[1]),
                prog.FindDecisionVariableIndex(var=x[1]))
            self.assertTrue(np.allclose(constraint.A(), [3, -1]))
            self.assertTrue(constraint.lower_bound(), -2)
            self.assertTrue(constraint.upper_bound(), np.inf)

        self.assertTrue(prog.linear_equality_constraints())
        for (i, binding) in enumerate(prog.linear_equality_constraints()):
            self.assertIsNone(constraint.gradient_sparsity_pattern())
            constraint = binding.evaluator()
            self.assertEqual(
                prog.FindDecisionVariableIndex(var=binding.variables()[0]),
                prog.FindDecisionVariableIndex(var=x[0]))
            self.assertEqual(
                prog.FindDecisionVariableIndex(var=binding.variables()[1]),
                prog.FindDecisionVariableIndex(var=x[1]))
            self.assertTrue(np.allclose(constraint.A(), [1, 2]))
            self.assertTrue(constraint.lower_bound(), 3)
            self.assertTrue(constraint.upper_bound(), 3)

        result = mp.Solve(prog)
        self.assertTrue(result.is_success())

        x_expected = np.array([1, 1])
        self.assertTrue(np.allclose(result.GetSolution(x), x_expected))

    def test_constraint_api(self):
        prog = mp.MathematicalProgram()
        x0, = prog.NewContinuousVariables(1, "x")
        c = prog.AddLinearConstraint(x0 >= 2).evaluator()
        ce = prog.AddLinearEqualityConstraint(2*x0, 1).evaluator()

        self.assertTrue(c.CheckSatisfied([2.], tol=1e-3))
        satisfied = c.CheckSatisfiedVectorized(
            np.array([1., 2., 3.]).reshape((1, 3)), tol=1e-3)
        self.assertEqual(satisfied, [False, True, True])
        self.assertFalse(c.CheckSatisfied([AutoDiffXd(1.)]))
        self.assertIsInstance(c.CheckSatisfied([x0]), sym.Formula)

        ce.set_description("my favorite constraint")
        self.assertEqual(ce.get_description(), "my favorite constraint")

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
        lc = prog.AddLinearCost([1], 2, [x0]).evaluator()
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

        result = mp.Solve(prog)
        self.assertTrue(np.allclose(result.GetSolution(x), x_expected))

        enum = zip(constraints, constraint_values_expected)
        for (constraint, value_expected) in enum:
            value = result.EvalBinding(constraint)
            self.assertTrue(np.allclose(value, value_expected))
            value = prog.EvalBinding(constraint, x_expected)
            self.assertTrue(np.allclose(value, value_expected))
            value = prog.EvalBindingVectorized(
                constraint,
                np.vstack((x_expected, x_expected)).T)
            a = np.vstack((value_expected, value_expected)).T
            self.assertTrue(np.allclose(
                value, np.vstack((value_expected, value_expected)).T))

        enum = zip(costs, cost_values_expected)
        for (cost, value_expected) in enum:
            value = result.EvalBinding(cost)
            self.assertTrue(np.allclose(value, value_expected))
            value = prog.EvalBinding(cost, x_expected)
            self.assertTrue(np.allclose(value, value_expected))
            value = prog.EvalBindingVectorized(
                cost,
                np.vstack((x_expected, x_expected)).T)
            self.assertTrue(np.allclose(
                value, np.vstack((value_expected, value_expected)).T))

        self.assertIsInstance(
            result.EvalBinding(costs[0]), np.ndarray)

        # Bindings for `Eval`.
        x_list = (float(1.), AutoDiffXd(1.), sym.Variable("x"))
        T_y_list = (float, AutoDiffXd, sym.Expression)
        evaluator = costs[0].evaluator()
        for x_i, T_y_i in zip(x_list, T_y_list):
            y_i = evaluator.Eval(x=[x_i, x_i])
            self.assertIsInstance(y_i[0], T_y_i)

    def test_get_binding_variable_values(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(3)
        binding1 = prog.AddBoundingBoxConstraint(-1, 1, x[0])
        binding2 = prog.AddLinearEqualityConstraint(x[1] + 2*x[2], 2)
        x_val = np.array([-2., 1., 2.])
        np.testing.assert_allclose(
            prog.GetBindingVariableValues(binding1, x_val), np.array([-2]))
        np.testing.assert_allclose(
            prog.GetBindingVariableValues(binding2, x_val), np.array([1, 2]))

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
        # d(0) + d(1) = 1
        prog = mp.MathematicalProgram()
        x = prog.NewIndeterminates(1, "x")
        self.assertEqual(prog.indeterminates_index()[x[0].get_id()], 0)
        poly = prog.NewFreePolynomial(sym.Variables(x), 1)
        (poly, binding) = prog.NewSosPolynomial(
            indeterminates=sym.Variables(x), degree=2)
        even_poly = prog.NewEvenDegreeFreePolynomial(sym.Variables(x), 2)
        odd_poly = prog.NewOddDegreeFreePolynomial(sym.Variables(x), 3)
        y = prog.NewIndeterminates(1, "y")
        self.assertEqual(prog.indeterminates_index()[y[0].get_id()], 1)
        (poly, binding) = prog.NewSosPolynomial(
            monomial_basis=(sym.Monomial(x[0]), sym.Monomial(y[0])))
        d = prog.NewContinuousVariables(2, "d")
        prog.AddSosConstraint(d[0]*x.dot(x))
        prog.AddSosConstraint(d[1]*x.dot(x), [sym.Monomial(x[0])])
        prog.AddLinearEqualityConstraint(d[0] + d[1] == 1)
        result = mp.Solve(prog)
        self.assertTrue(result.is_success())
        result.GetSolution(poly)

        (poly, Q_oo, Q_ee) = prog.NewEvenDegreeSosPolynomial(
            indeterminates=sym.Variables(x), degree=2)
        (poly, Q_oo, Q_ee) = prog.NewEvenDegreeSdsosPolynomial(
            indeterminates=sym.Variables(x), degree=2)
        (poly, Q_oo, Q_ee) = prog.NewEvenDegreeDsosPolynomial(
            indeterminates=sym.Variables(x), degree=2)

    def test_make_polynomial(self):
        prog = mp.MathematicalProgram()
        x = prog.NewIndeterminates(1, "x")[0]
        a = prog.NewContinuousVariables(1, "a")[0]
        # e = (a + 1)x² + 2ax + 3a.
        e = (a + 1) * (x * x) + (2 * a) * x + 3 * a

        # We create a polynomial of `e` via MakePolynomial.
        p = prog.MakePolynomial(e)
        # Check its indeterminates and decision variables are correctly set,
        self.assertEqual(p.indeterminates().size(), 1)
        self.assertTrue(p.indeterminates().include(x))
        self.assertEqual(p.decision_variables().size(), 1)
        self.assertTrue(p.decision_variables().include(a))
        # Check if it holds the same expression when converted back to
        # symbolic expression.
        self.assertTrue(p.ToExpression().EqualTo(e))

    def test_reparse(self):
        prog = mp.MathematicalProgram()
        x = prog.NewIndeterminates(1, "x")[0]
        a = prog.NewContinuousVariables(1, "a")[0]
        e = (a + 1) * (x * x) + (2 * a) * x + 3 * a

        # p = (x^2 + 2x + 3)a + x^2 with indeterminates {a}.
        p = sym.Polynomial(e, [a])
        self.assertEqual(p.TotalDegree(), 1)

        # p = (a + 1)x² + 2ax + 3a with indeterminates {x}.
        prog.Reparse(p)
        self.assertEqual(p.TotalDegree(), 2)

    def test_equality_between_polynomials(self):
        prog = mp.MathematicalProgram()
        x = prog.NewIndeterminates(1, "x")
        a = prog.NewContinuousVariables(2, "a")
        prog.AddEqualityConstraintBetweenPolynomials(sym.Polynomial(
            2 * a[0] * x[0] + a[1] + 2, x), sym.Polynomial(2 * x[0] + 4, x))
        result = mp.Solve(prog)
        a_val = result.GetSolution(a)
        self.assertAlmostEqual(a_val[0], 1)
        self.assertAlmostEqual(a_val[1], 2)

    def test_log_determinant(self):
        # Find the minimal ellipsoid that covers some given points.
        prog = mp.MathematicalProgram()
        X = prog.NewSymmetricContinuousVariables(2)
        pts = np.array([[1, 1], [1, -1], [-1, 1]])
        for i in range(3):
            pt = pts[i, :]
            prog.AddLinearConstraint(pt.dot(X.dot(pt)) <= 1)
        prog.AddMaximizeLogDeterminantSymmetricMatrixCost(X)
        result = mp.Solve(prog)
        self.assertTrue(result.is_success())

    def test_maximize_geometric_mean(self):
        # Find the smallest axis-algined ellipsoid that covers some given
        # points.
        prog = mp.MathematicalProgram()
        a = prog.NewContinuousVariables(2)
        pts = np.array([[1, 1], [1, -1], [-1, 1]])
        for i in range(3):
            pt = pts[i, :]
            prog.AddLinearConstraint(pt.dot(a * pt) <= 1)
        prog.AddMaximizeGeometricMeanCost(a, 1)
        result = mp.Solve(prog)
        self.assertTrue(result.is_success())

    def test_max_geometric_mean_trivial(self):
        # Solve the trivial problem.
        # max (2x+3)*(3x+2)
        # s.t 2x+3 >= 0
        #     3x+2 >= 0
        #     x <= 10
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(1)
        prog.AddLinearConstraint(x[0] <= 10)
        A = np.array([2, 3])
        b = np.array([3, 2])
        prog.AddMaximizeGeometricMeanCost(A, b, x)
        result = mp.Solve(prog)
        self.assertTrue(result.is_success())

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
        prog.AddLinearEqualityConstraint(
            2 * x[:2] + np.array([0, 1]), np.array([3, 2]))

    def test_constraint_gradient_sparsity(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")

        def cost(x):
            return x[0]**2

        def constraint(x):
            return x[1] ** 2

        cost_binding = prog.AddCost(cost, vars=x)
        constraint_binding = prog.AddConstraint(constraint, [0], [1], vars=x)
        cost_evaluator = cost_binding.evaluator()
        constraint_evaluator = constraint_binding.evaluator()
        self.assertIsNone(cost_evaluator.gradient_sparsity_pattern())
        self.assertIsNone(constraint_evaluator.gradient_sparsity_pattern())
        # Now set the sparsity
        cost_evaluator.SetGradientSparsityPattern([(0, 0)])
        self.assertEqual(cost_evaluator.gradient_sparsity_pattern(), [(0, 0)])
        constraint_binding.evaluator().SetGradientSparsityPattern([(0, 1)])
        self.assertEqual(
            constraint_evaluator.gradient_sparsity_pattern(),
            [(0, 1)])

    def test_pycost_and_pyconstraint(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(1, 'x')

        def cost(x):
            return (x[0]-1.)*(x[0]-1.)

        def constraint(x):
            return x

        cost_binding = prog.AddCost(cost, vars=x)
        constraint_binding = prog.AddConstraint(
            constraint, lb=[0.], ub=[2.], vars=x)
        result = mp.Solve(prog)
        xstar = result.GetSolution(x)
        self.assertAlmostEqual(xstar[0], 1.)

        # Verify that they can be evaluated.
        self.assertAlmostEqual(cost_binding.evaluator().Eval(xstar), 0.)
        self.assertAlmostEqual(constraint_binding.evaluator().Eval(xstar), 1.)
        self.assertEqual(len(prog.generic_constraints()), 1)
        self.assertEqual(
            prog.generic_constraints()[0].evaluator(),
            constraint_binding.evaluator())
        self.assertEqual(len(prog.generic_costs()), 1)
        self.assertEqual(
            prog.generic_costs()[0].evaluator(), cost_binding.evaluator())

    def get_different_scalar_type(self, T):
        # Gets U such that U != T.
        next_index = SCALAR_TYPES.index(T) + 1
        U = SCALAR_TYPES[next_index % len(SCALAR_TYPES)]
        self.assertNotEqual(U, T)
        return U

    def test_pycost_wrap_error(self):
        """Tests for checks using PyFunctionCost::Wrap."""
        # TODO(eric.cousineau): It would be nice to not need a
        # MathematicalProgram to test these.

        def user_cost_bad_shape(x):
            # WARNING: This should return a scalar, not a vector!
            return x

        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(1, 'x')
        binding_bad_shape = prog.AddCost(user_cost_bad_shape, vars=x)

        for T in SCALAR_TYPES:
            array_T = np.vectorize(T)
            x0 = array_T([0.])
            x0_bad = array_T([0., 1.])
            # Bad input (before function is called).
            if kDrakeAssertIsArmed:
                # See note in `WrapUserFunc`.
                input_error_cls = SystemExit
                input_error_expected = (
                    "x.rows() == num_vars_ || num_vars_ == Eigen::Dynamic")
            else:
                input_error_cls = RuntimeError
                input_error_expected = (
                    "PyFunctionCost: Input must be of .ndim = 1 or 2 (vector) "
                    "and .size = 1. Got .ndim = 1 and .size = 2 instead.")
            with self.assertRaises(input_error_cls) as cm:
                binding_bad_shape.evaluator().Eval(x0_bad)
            self.assertIn(input_error_expected, str(cm.exception))
            # Bad output shape.
            with self.assertRaises(RuntimeError) as cm:
                binding_bad_shape.evaluator().Eval(x0)
            self.assertEqual(
                str(cm.exception),
                "PyFunctionCost: Output must be of .ndim = 0 (scalar) and "
                ".size = 1. Got .ndim = 1 and .size = 1 instead.")

            # Bad output dtype.
            U = self.get_different_scalar_type(T)

            def user_cost_bad_dtype(x):
                # WARNING: This should return the same dtype as x!
                return U(0.)

            binding_bad_dtype = prog.AddCost(user_cost_bad_dtype, vars=x)
            with self.assertRaises(RuntimeError) as cm:
                binding_bad_dtype.evaluator().Eval(x0)
            self.assertEqual(
                str(cm.exception),
                f"PyFunctionCost: Output must be of scalar type {T.__name__}. "
                f"Got {U.__name__} instead.")

    def test_pyconstraint_wrap_error(self):
        """Tests for checks using PyFunctionConstraint::Wrap."""
        # TODO(eric.cousineau): It would be nice to not need a
        # MathematicalProgram to test these.

        def user_constraint_bad_shape(x):
            # WARNING: This should return a vector, not a scalar!
            return x[0]

        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(1, 'x')
        binding_bad_shape = prog.AddConstraint(
            user_constraint_bad_shape, lb=[0.], ub=[2.], vars=x)

        for T in SCALAR_TYPES:
            array_T = np.vectorize(T)
            x0 = array_T([0.])
            x0_bad = array_T([0., 1.])
            # Bad input (before function is called).
            if kDrakeAssertIsArmed:
                # See note in `WrapUserFunc`.
                input_error_cls = SystemExit
                input_error_expected = (
                    "x.rows() == num_vars_ || num_vars_ == Eigen::Dynamic")
            else:
                input_error_cls = RuntimeError
                input_error_expected = (
                    "PyFunctionConstraint: Input must be of .ndim = 1 or 2 "
                    "(vector) and .size = 1. Got .ndim = 1 and .size = 2 "
                    "instead.")
            with self.assertRaises(input_error_cls) as cm:
                binding_bad_shape.evaluator().Eval(x0_bad)
            self.assertIn(input_error_expected, str(cm.exception))
            # Bad output.
            with self.assertRaises(RuntimeError) as cm:
                binding_bad_shape.evaluator().Eval(x0)
            self.assertEqual(
                str(cm.exception),
                "PyFunctionConstraint: Output must be of .ndim = 1 or 2 "
                "(vector) and .size = 1. Got .ndim = 0 and .size = 1 instead.")

            # Bad output dtype.
            U = self.get_different_scalar_type(T)

            def user_constraint_bad_dtype(x):
                # WARNING: This should return the same dtype as x!
                return [U(0.)]

            binding_bad_dtype = prog.AddConstraint(
                user_constraint_bad_dtype, lb=[0.], ub=[2.], vars=x)
            with self.assertRaises(RuntimeError) as cm:
                binding_bad_dtype.evaluator().Eval(x0)
            self.assertEqual(
                str(cm.exception),
                f"PyFunctionConstraint: Output must be of scalar type "
                f"{T.__name__}. Got {U.__name__} instead.")

    def test_addcost_symbolic(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(1, 'x')
        prog.AddCost((x[0]-1.)**2)
        prog.AddConstraint(0 <= x[0])
        prog.AddConstraint(x[0] <= 2)
        result = mp.Solve(prog)
        self.assertAlmostEqual(result.GetSolution(x)[0], 1.)

    def test_addconstraint_matrix(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(1, 'x')
        prog.AddConstraint(np.array([[x[0] <= 2], [x[0] >= -2]]))
        result = mp.Solve(prog)
        self.assertTrue(result.GetSolution(x)[0] <= 2)
        self.assertTrue(result.GetSolution(x)[0] >= -2)

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
        # The default initial guess is [0, 0, 0]. This initial guess is bad
        # because LorentzConeConstraint with eval_type=kConvex is not
        # differentiable at [0, 0, 0]. Use initial guess [0.5, 0.5, 0.5]
        # instead.
        result = mp.Solve(prog, [0.5, 0.5, 0.5])
        self.assertTrue(result.is_success())

        # Check answer
        x_expected = np.array([1-2**(-0.5), 1-2**(-0.5)])
        self.assertTrue(np.allclose(result.GetSolution(x), x_expected))

    def test_add_lorentz_cone_constraint(self):
        # Call AddLorentzConeConstraint, make sure no error is thrown.
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(3)

        prog.AddLorentzConeConstraint(
            v=np.array([x[0]+1, x[1]+x[2], 2*x[1]]))
        prog.AddLorentzConeConstraint(
            linear_expression=x[0] + x[1] + 1,
            quadratic_expression=x[0]*x[0] + x[1] * x[1] + 2 * x[0] * x[1] + 1,
            tol=0.)
        A = np.array([[1, 0], [0, 1], [1, 0], [0, 0]])
        b = np.array([1, 1, 0, 2])
        constraint = prog.AddLorentzConeConstraint(A=A, b=b, vars=x[:2])
        np.testing.assert_allclose(
            constraint.evaluator().A().todense(), A)
        np.testing.assert_allclose(constraint.evaluator().b(), b)

    def test_add_rotated_lorentz_cone_constraint(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(3)

        A = np.array([[1, 0], [1, 1], [1, 0], [0, 1], [0, 0]])
        b = np.array([1, 0, 1, 0, 2])
        constraint = prog.AddRotatedLorentzConeConstraint(A=A, b=b, vars=x[:2])
        np.testing.assert_allclose(
            constraint.evaluator().A().todense(), A)
        np.testing.assert_allclose(constraint.evaluator().b(), b)

        prog.AddRotatedLorentzConeConstraint(
            v=[x[0]+1, x[0]+x[1], x[0], x[2]+1, 2])
        constraint = prog.AddRotatedLorentzConeConstraint(
            linear_expression1=x[0]+1, linear_expression2=x[0]+x[1],
            quadratic_expression=x[0]*x[0] + 2*x[0] + x[1]*x[1] + 5)

    def test_solver_options(self):
        prog = mp.MathematicalProgram()

        prog.SetSolverOption(SolverType.kGurobi, "double_key", 1.0)
        prog.SetSolverOption(GurobiSolver().solver_id(), "int_key", 2)
        prog.SetSolverOption(SolverType.kGurobi, "string_key", "3")

        options = prog.GetSolverOptions(SolverType.kGurobi)
        self.assertDictEqual(
            options, {"double_key": 1.0, "int_key": 2, "string_key": "3"})
        options = prog.GetSolverOptions(GurobiSolver().solver_id())
        self.assertDictEqual(
            options, {"double_key": 1.0, "int_key": 2, "string_key": "3"})

        # For now, just make sure the constructor exists.  Once we bind more
        # accessors, we can test them here.
        options_object = SolverOptions()
        solver_id = SolverId("dummy")
        self.assertEqual(solver_id.name(), "dummy")
        options_object.SetOption(solver_id, "double_key", 1.0)
        options_object.SetOption(solver_id, "int_key", 2)
        options_object.SetOption(solver_id, "string_key", "3")
        options = options_object.GetOptions(solver_id)
        self.assertDictEqual(
            options, {"double_key": 1.0, "int_key": 2, "string_key": "3"})

        prog.SetSolverOptions(options_object)
        prog_options = prog.GetSolverOptions(solver_id)
        self.assertDictEqual(
            prog_options, {"double_key": 1.0, "int_key": 2, "string_key": "3"})

    def test_infeasible_constraints(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(1)
        result = mp.Solve(prog)

        infeasible = result.GetInfeasibleConstraints(prog)
        self.assertEqual(len(infeasible), 0)

        infeasible = result.GetInfeasibleConstraints(prog, tol=1e-4)
        self.assertEqual(len(infeasible), 0)

        infeasible_names = result.GetInfeasibleConstraintNames(
                prog=prog, tol=1e-4)
        self.assertEqual(len(infeasible_names), 0)

    def test_add_indeterminates_and_decision_variables(self):
        prog = mp.MathematicalProgram()
        x0 = sym.Variable("x0")
        x1 = sym.Variable("x1")
        a0 = sym.Variable("a0")
        a1 = sym.Variable("a1")
        prog.AddIndeterminates(np.array([x0, x1]))
        prog.AddDecisionVariables(np.array([a0, a1]))
        numpy_compare.assert_equal(prog.decision_variables()[0], a0)
        numpy_compare.assert_equal(prog.decision_variables()[1], a1)
        numpy_compare.assert_equal(prog.indeterminates()[0], x0)
        numpy_compare.assert_equal(prog.indeterminate(1), x1)


class DummySolverInterface(SolverInterface):
    def __init__(self):
        SolverInterface.__init__(self)

    def available(self):
        return True

    def enabled(self):
        return True

    def solver_id(self):
        return SolverId("dummy")

    def Solve(self, prog, initial_guess, solver_options, result):
        raise Exception("Dummy solver cannot solve")

    def AreProgramAttributesSatisfied(self, prog):
        return True


class TestSolverInterface(unittest.TestCase):
    def test_dummy_solver_interface(self):
        solver = DummySolverInterface()
        self.assertTrue(solver.available())
        self.assertTrue(solver.enabled())
        self.assertEqual(solver.solver_id().name(), "dummy")
        self.assertIsInstance(solver, SolverInterface)
        prog = mp.MathematicalProgram()
        result = mp.MathematicalProgramResult()
        with self.assertRaises(Exception) as context:
            solver.Solve(prog, None, None, result)
        self.assertTrue("Dummy solver cannot solve" in str(context.exception))
        self.assertIsInstance(result, mp.MathematicalProgramResult)
        self.assertTrue(solver.AreProgramAttributesSatisfied(prog))
        with self.assertRaises(Exception) as context:
            result2 = solver.Solve(prog)
            self.assertIsInstance(result2, mp.MathematicalProgramResult)
