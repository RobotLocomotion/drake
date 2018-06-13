from __future__ import print_function, absolute_import

from pydrake.solvers import mathematicalprogram as mp

import numpy as np
import unittest
import warnings

import pydrake
import pydrake.symbolic as sym


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
            # Linaer equality
            prog.AddLinearConstraint(x[0] + 2 * x[1] == 3)]

        # TODO(eric.cousineau): Add constant terms
        self.costs = [prog.AddLinearCost(x[0] + x[1]),
                      prog.AddQuadraticCost(0.5 * (x[0]**2 + x[1]**2))]


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
        self.assertIsNone(prog.GetSolverId())
        result = prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)
        self.assertIsNotNone(prog.GetSolverId().name())

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
        # Redundant cost just to check the spelling.
        prog.AddQuadraticErrorCost(vars=x, Q=np.eye(2),
                                   x_desired=np.zeros(2))
        result = prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)

        x_expected = np.array([1, 1])
        self.assertTrue(np.allclose(prog.GetSolution(x), x_expected))

    def test_symbolic_qp(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddConstraint(x[0], 1., 100.)
        prog.AddConstraint(x[1] >= 1)
        prog.AddQuadraticCost(x[0]**2 + x[1]**2)
        result = prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)

        x_expected = np.array([1, 1])
        self.assertTrue(np.allclose(prog.GetSolution(x), x_expected))

    def test_bindings(self):
        qp = TestQP()
        prog = qp.prog
        x = qp.x

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

        result = prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)

        x_expected = np.array([1, 1])
        self.assertTrue(np.allclose(prog.GetSolution(x), x_expected))

        # Test deprecated method.
        with warnings.catch_warnings(record=True) as w:
            c = binding.constraint()
            self.assertEqual(len(w), 1)

    def test_constraint_api(self):
        prog = mp.MathematicalProgram()
        x0, = prog.NewContinuousVariables(1, "x")
        c = prog.AddLinearConstraint(x0 >= 2).evaluator()

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

    def test_eval_binding(self):
        qp = TestQP()
        prog = qp.prog

        x = qp.x
        x_expected = np.array([1., 1.])

        costs = qp.costs
        cost_values_expected = [2., 1.]
        constraints = qp.constraints
        constraint_values_expected = [1., 1., 2., 3.]

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
        # Just check spelling.
        y = prog.NewIndeterminates(2, 2, "y")

    def test_sdp(self):
        prog = mp.MathematicalProgram()
        S = prog.NewSymmetricContinuousVariables(3, "S")
        prog.AddLinearConstraint(S[0, 1] >= 1)
        prog.AddPositiveSemidefiniteConstraint(S)
        prog.AddPositiveSemidefiniteConstraint(S+S)
        prog.AddLinearCost(np.trace(S))
        result = prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)
        S = prog.GetSolution(S)
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
        result = prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)

        # Test SubstituteSolution(sym.Expression)
        # TODO(eric.cousineau): Expose `SymbolicTestCase` so that other tests
        # can use the assertion utilities.
        self.assertEqual(
            prog.SubstituteSolution(d[0] + d[1]).Evaluate(),
            prog.GetSolution(d[0]) + prog.GetSolution(d[1]))
        # Test SubstituteSolution(sym.Polynomial)
        poly = d[0]*x.dot(x)
        poly_sub_actual = prog.SubstituteSolution(
            sym.Polynomial(poly, sym.Variables(x)))
        poly_sub_expected = sym.Polynomial(
            prog.SubstituteSolution(d[0])*x.dot(x), sym.Variables(x))
        # TODO(soonho): At present, these must be converted to `Expression` to
        # compare, because as `Polynomial`s the comparison fails with
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
        result = prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)
        self.assertIsInstance(binding.evaluator(),
                              mp.LinearComplementarityConstraint)

    def test_bounding_box(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(2, 'x')
        lb = [0., 0.]
        ub = [1., 1.]
        prog.AddBoundingBoxConstraint(lb, ub, x)
        prog.AddBoundingBoxConstraint(0., 1., x[0])
        prog.AddBoundingBoxConstraint(0., 1., x)

    def test_pycost_and_pyconstraint(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(1, 'x')

        def cost(x):
            return (x[0]-1.)*(x[0]-1.)

        def constraint(x):
            return x

        prog.AddCost(cost, x)
        prog.AddConstraint(constraint, [0.], [2.], x)
        prog.Solve()
        self.assertAlmostEqual(prog.GetSolution(x)[0], 1.)

    def test_addcost_symbolic(self):
        prog = mp.MathematicalProgram()
        x = prog.NewContinuousVariables(1, 'x')
        prog.AddCost((x[0]-1.)**2)
        prog.AddConstraint(0 <= x[0])
        prog.AddConstraint(x[0] <= 2)
        prog.Solve()
        self.assertAlmostEqual(prog.GetSolution(x)[0], 1.)

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
        for i in xrange(count):
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
        result = prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)

        # Check answer
        x_expected = np.array([1-2**(-0.5), 1-2**(-0.5)])
        self.assertTrue(np.allclose(prog.GetSolution(x), x_expected))
