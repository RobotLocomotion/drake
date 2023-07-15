import unittest

import numpy as np

import pydrake.solvers as mp
import pydrake.symbolic as sym


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
        self.assertFalse(cost.is_convex())

        cost = mp.QuadraticCost(Q, b, c, is_convex=False)
        self.assertFalse(cost.is_convex())

        cost = mp.QuadraticCost(np.array([[1., 2.], [2., 6.]]), b, c)
        self.assertTrue(cost.is_convex())

    def test_l1norm_cost(self):
        A = np.array([[1., 2.], [-.4, .7]])
        b = np.array([0.5, -.4])
        cost = mp.L1NormCost(A=A, b=b)
        np.testing.assert_allclose(cost.A(), A)
        np.testing.assert_allclose(cost.b(), b)
        cost.UpdateCoefficients(new_A=2*A, new_b=2*b)
        np.testing.assert_allclose(cost.A(), 2*A)
        np.testing.assert_allclose(cost.b(), 2*b)

    def test_l2norm_cost(self):
        A = np.array([[1., 2.], [-.4, .7]])
        b = np.array([0.5, -.4])
        cost = mp.L2NormCost(A=A, b=b)
        np.testing.assert_allclose(cost.A(), A)
        np.testing.assert_allclose(cost.b(), b)
        cost.UpdateCoefficients(new_A=2*A, new_b=2*b)
        np.testing.assert_allclose(cost.A(), 2*A)
        np.testing.assert_allclose(cost.b(), 2*b)

    def test_linfnorm_cost(self):
        A = np.array([[1., 2.], [-.4, .7]])
        b = np.array([0.5, -.4])
        cost = mp.LInfNormCost(A=A, b=b)
        np.testing.assert_allclose(cost.A(), A)
        np.testing.assert_allclose(cost.b(), b)
        cost.UpdateCoefficients(new_A=2*A, new_b=2*b)
        np.testing.assert_allclose(cost.A(), 2*A)
        np.testing.assert_allclose(cost.b(), 2*b)

    def test_perspective_quadratic_cost(self):
        A = np.array([[1., 2.], [-.4, .7]])
        b = np.array([0.5, -.4])
        cost = mp.PerspectiveQuadraticCost(A=A, b=b)
        np.testing.assert_allclose(cost.A(), A)
        np.testing.assert_allclose(cost.b(), b)
        cost.UpdateCoefficients(new_A=2*A, new_b=2*b)
        np.testing.assert_allclose(cost.A(), 2*A)
        np.testing.assert_allclose(cost.b(), 2*b)

    def test_expression_cost(self):
        x = sym.Variable("x")
        y = sym.Variable("y")
        e = np.sin(x) + y
        cost = mp.ExpressionCost(e=e)
        self.assertTrue(e.EqualTo(cost.expression()))
        self.assertEqual(sym.Variables(cost.vars()), sym.Variables([x, y]))


class TestConstraints(unittest.TestCase):
    def test_bounding_box_constraint(self):
        constraint = mp.BoundingBoxConstraint(
            lb=np.array([1., 2.]), ub=np.array([2., 3.]))
        np.testing.assert_array_equal(
            constraint.lower_bound(), np.array([1., 2.]))
        np.testing.assert_array_equal(
            constraint.upper_bound(), np.array([2., 3.]))

    def test_quadratic_constraint(self):
        hessian_type = mp.QuadraticConstraint.HessianType.kPositiveSemidefinite
        constraint = mp.QuadraticConstraint(
            Q0=np.eye(2), b=np.array([1, 2.]), lb=-np.inf, ub=1.,
            hessian_type=hessian_type)
        np.testing.assert_array_equal(constraint.Q(), np.eye(2))
        np.testing.assert_array_equal(constraint.b(), np.array([1, 2.]))
        self.assertEqual(constraint.hessian_type(), hessian_type)
        self.assertTrue(constraint.is_convex())
        hessian_type = mp.QuadraticConstraint.HessianType.kNegativeSemidefinite
        constraint.UpdateCoefficients(
            new_Q=-np.eye(2), new_b=np.array([1., -1.]),
            hessian_type=hessian_type)
        self.assertEqual(constraint.hessian_type(), hessian_type)
        self.assertFalse(constraint.is_convex())
        constraint.UpdateCoefficients(
            new_Q=np.array([[1, 0], [0, -1.]]), new_b=np.array([1., -1]))
        hessian_type = mp.QuadraticConstraint.HessianType.kIndefinite
        self.assertEqual(constraint.hessian_type(), hessian_type)

    def test_positive_semidefinite_constraint(self):
        constraint = mp.PositiveSemidefiniteConstraint(rows=3)
        self.assertEqual(constraint.matrix_rows(), 3)

    def test_linear_matrix_inequality_constraint(self):
        constraint = mp.LinearMatrixInequalityConstraint(
            F=[np.eye(3), 2 * np.eye(3), np.ones((3, 3))],
            symmetry_tolerance=1E-12)
        self.assertEqual(constraint.matrix_rows(), 3)

    def test_expression_constraint(self):
        x = sym.Variable("x")
        y = sym.Variable("y")
        v = [np.sin(x) + y, x + np.cos(y)]
        lb = [-1.2, -2.4]
        ub = [53.3, 2.35]
        constraint = mp.ExpressionConstraint(v=v, lb=lb, ub=ub)
        self.assertTrue(v[0].EqualTo(constraint.expressions()[0]))
        self.assertTrue(v[1].EqualTo(constraint.expressions()[1]))
        self.assertEqual(
            sym.Variables(constraint.vars()), sym.Variables([x, y]))
        np.testing.assert_array_equal(lb, constraint.lower_bound())
        np.testing.assert_array_equal(ub, constraint.upper_bound())

    def test_lorentz_cone_constraint(self):
        A = np.array([[1, 2], [-1, -3], [2, 3.]])
        b = np.array([2., 3., 4.])
        constraint = mp.LorentzConeConstraint(
            A=A, b=b,
            eval_type=mp.LorentzConeConstraint.EvalType.kConvexSmooth)
        np.testing.assert_array_equal(constraint.A().todense(), A)
        np.testing.assert_array_equal(constraint.b(), b)
        self.assertEqual(
            constraint.eval_type(),
            mp.LorentzConeConstraint.EvalType.kConvexSmooth)
        constraint.UpdateCoefficients(new_A=2 * A, new_b=3 * b)
        np.testing.assert_array_equal(constraint.A().todense(), 2 * A)
        np.testing.assert_array_equal(constraint.b(), 3 * b)

    def test_rotated_lorentz_cone_constraint(self):
        A = np.array(
            [[1., 2., 3.], [4., 5., 6.], [7., 8., 9.], [10., 11., 12.]])
        b = np.array([1., 2., 3, 4])
        constraint = mp.RotatedLorentzConeConstraint(A=A, b=b)
        np.testing.assert_array_equal(constraint.A().todense(), A)
        np.testing.assert_array_equal(constraint.b(), b)
        constraint.UpdateCoefficients(new_A=2 * A, new_b=3 * b)
        np.testing.assert_array_equal(constraint.A().todense(), 2 * A)
        np.testing.assert_array_equal(constraint.b(), 3 * b)

    def test_binding_instantiations(self):
        # Patterned spelling.
        cls_list = [
            mp.EvaluatorBase,
            mp.Constraint,
            mp.LinearConstraint,
            mp.LorentzConeConstraint,
            mp.RotatedLorentzConeConstraint,
            mp.LinearEqualityConstraint,
            mp.BoundingBoxConstraint,
            mp.PositiveSemidefiniteConstraint,
            mp.LinearMatrixInequalityConstraint,
            mp.LinearComplementarityConstraint,
            mp.ExponentialConeConstraint,
            mp.Cost,
            mp.LinearCost,
            mp.QuadraticCost,
            mp.L1NormCost,
            mp.L2NormCost,
            mp.LInfNormCost,
            mp.VisualizationCallback,
        ]
        for cls in cls_list:
            mp.Binding[cls]
