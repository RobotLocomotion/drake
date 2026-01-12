import typing
import unittest
import weakref

import numpy as np
import scipy.sparse

from pydrake.autodiffutils import InitializeAutoDiff
import pydrake.solvers as mp
import pydrake.solvers._testing as mp_testing
import pydrake.symbolic as sym


class TestCost(unittest.TestCase):
    def test_linear_cost(self):
        a = np.array([1.0, 2.0])
        b = 0.5
        cost = mp.LinearCost(a, b)
        np.testing.assert_allclose(cost.a(), a)
        self.assertEqual(cost.b(), b)
        cost.UpdateCoefficients(new_a=[2, 3.0], new_b=1)
        np.testing.assert_allclose(cost.a(), [2, 3.0])
        self.assertEqual(cost.b(), 1)

        cost.update_coefficient_entry(i=0, val=4)
        np.testing.assert_allclose(cost.a(), [4, 3])
        self.assertEqual(cost.b(), 1)

        cost.update_constant_term(new_b=2)
        self.assertEqual(cost.b(), 2)

    def test_quadratic_cost(self):
        Q = np.array([[1.0, 2.0], [2.0, 3.0]])
        b = np.array([3.0, 4.0])
        c = 0.4
        cost = mp.QuadraticCost(Q, b, c)
        np.testing.assert_allclose(cost.Q(), Q)
        np.testing.assert_allclose(cost.b(), b)
        self.assertEqual(cost.c(), c)
        self.assertFalse(cost.is_convex())

        cost = mp.QuadraticCost(Q, b, c, is_convex=False)
        self.assertFalse(cost.is_convex())

        cost = mp.QuadraticCost(np.array([[1.0, 2.0], [2.0, 6.0]]), b, c)
        self.assertTrue(cost.is_convex())

        cost.UpdateCoefficients(
            new_Q=np.array([[1, 3], [3, 6.0]]),
            new_b=np.array([2, 4.0]),
            new_c=1,
            is_convex=False,
        )
        np.testing.assert_allclose(cost.Q(), np.array([[1, 3], [3, 6]]))
        np.testing.assert_allclose(cost.b(), np.array([2, 4.0]))
        self.assertEqual(cost.c(), 1)

        cost.UpdateHessianEntry(i=0, j=1, val=1, is_hessian_psd=None)
        np.testing.assert_allclose(cost.Q(), np.array([[1, 1], [1, 6]]))
        self.assertTrue(cost.is_convex())
        cost.update_linear_coefficient_entry(i=1, val=5)
        np.testing.assert_allclose(cost.b(), np.array([2, 5.0]))
        cost.update_constant_term(new_c=2)
        self.assertEqual(cost.c(), 2)

        # User-specify is_hessian_psd.
        cost.UpdateHessianEntry(i=0, j=0, val=20, is_hessian_psd=True)
        self.assertTrue(cost.is_convex())

    def test_l1norm_cost(self):
        A = np.array([[1.0, 2.0], [-0.4, 0.7]])
        b = np.array([0.5, -0.4])
        cost = mp.L1NormCost(A=A, b=b)
        np.testing.assert_allclose(cost.A(), A)
        np.testing.assert_allclose(cost.b(), b)
        cost.UpdateCoefficients(new_A=2 * A, new_b=2 * b)
        np.testing.assert_allclose(cost.A(), 2 * A)
        np.testing.assert_allclose(cost.b(), 2 * b)
        cost.update_A_entry(i=0, j=1, val=0.5)
        np.testing.assert_allclose(cost.A(), np.array([[2, 0.5], [-0.8, 1.4]]))
        cost.update_b_entry(i=0, val=1)
        np.testing.assert_allclose(cost.b(), np.array([1, -0.8]))

    def test_l2norm_cost(self):
        A = np.array([[1.0, 2.0], [-0.4, 0.7]])
        b = np.array([0.5, -0.4])
        cost = mp.L2NormCost(A=A, b=b)
        np.testing.assert_allclose(cost.get_sparse_A().todense(), A)
        np.testing.assert_allclose(cost.GetDenseA(), A)
        np.testing.assert_allclose(cost.b(), b)
        cost.UpdateCoefficients(new_A=2 * A, new_b=2 * b)
        np.testing.assert_allclose(cost.b(), 2 * b)

    def test_linfnorm_cost(self):
        A = np.array([[1.0, 2.0], [-0.4, 0.7]])
        b = np.array([0.5, -0.4])
        cost = mp.LInfNormCost(A=A, b=b)
        np.testing.assert_allclose(cost.A(), A)
        np.testing.assert_allclose(cost.b(), b)
        cost.UpdateCoefficients(new_A=2 * A, new_b=2 * b)
        np.testing.assert_allclose(cost.A(), 2 * A)
        np.testing.assert_allclose(cost.b(), 2 * b)
        cost.update_A_entry(i=0, j=1, val=0.5)
        np.testing.assert_allclose(cost.A(), np.array([[2, 0.5], [-0.8, 1.4]]))
        cost.update_b_entry(i=0, val=1)
        np.testing.assert_allclose(cost.b(), np.array([1, -0.8]))

    def test_perspective_quadratic_cost(self):
        A = np.array([[1.0, 2.0], [-0.4, 0.7]])
        b = np.array([0.5, -0.4])
        cost = mp.PerspectiveQuadraticCost(A=A, b=b)
        np.testing.assert_allclose(cost.A(), A)
        np.testing.assert_allclose(cost.b(), b)
        cost.UpdateCoefficients(new_A=2 * A, new_b=2 * b)
        np.testing.assert_allclose(cost.A(), 2 * A)
        np.testing.assert_allclose(cost.b(), 2 * b)
        cost.update_A_entry(i=0, j=1, val=0.5)
        np.testing.assert_allclose(cost.A(), np.array([[2, 0.5], [-0.8, 1.4]]))
        cost.update_b_entry(i=0, val=1)
        np.testing.assert_allclose(cost.b(), np.array([1, -0.8]))

    def test_expression_cost(self):
        x = sym.Variable("x")
        y = sym.Variable("y")
        e = np.sin(x) + y
        cost = mp.ExpressionCost(e=e)
        self.assertTrue(e.EqualTo(cost.expression()))
        self.assertEqual(sym.Variables(cost.vars()), sym.Variables([x, y]))

    def test_cost_python_wrapper_lost(self):
        # Ensure costs' python wrappers are kept alive when bound. See
        # issue #20131 for original problem description.

        # Make some objects in a function to let most of them be deleted at
        # scope exit. We will test if the contents of the first return value
        # ("keepers") succeed in keeping alive the objects tracked by "spies".
        def make_object_graph():
            spies = []
            x = sym.Variable("x")
            y = sym.Variable("y")
            e = np.sin(x) + y
            cost = mp.ExpressionCost(e=e)
            binding = mp.Binding[mp.ExpressionCost](cost, cost.vars())
            spies.append(weakref.finalize(cost, lambda: None))
            return binding, spies

        keeper, spies = make_object_graph()
        self.assertTrue(all(spy.alive for spy in spies))

    def test_to_latex(self):
        x = sym.Variable("x")
        y = sym.Variable("y")
        e = np.sin(x) + y
        cost = mp.ExpressionCost(e=e)
        self.assertEqual(
            cost.ToLatex(vars=cost.vars(), precision=1), "(y + \\sin{x})"
        )
        binding = mp.Binding[mp.ExpressionCost](cost, cost.vars())
        self.assertEqual(binding.ToLatex(precision=1), "(y + \\sin{x})")

    def test_binding_eq(self):
        x = sym.Variable("x")
        y = sym.Variable("y")
        z = sym.Variable("z")
        e1 = np.sin(x) + y
        e2 = np.cos(x) + y
        e3 = np.sin(z) + y

        cost1 = mp.ExpressionCost(e=e1)
        cost1_binding1 = mp.Binding[mp.ExpressionCost](cost1, cost1.vars())
        cost1_binding2 = mp.Binding[mp.ExpressionCost](cost1, cost1.vars())

        cost2 = mp.ExpressionCost(e=e2)
        cost2_binding = mp.Binding[mp.ExpressionCost](cost2, cost2.vars())

        cost3 = mp.ExpressionCost(e=e3)
        cost3_binding = mp.Binding[mp.ExpressionCost](cost3, cost3.vars())

        self.assertTrue(cost1_binding1 == cost1_binding1)
        self.assertTrue(cost1_binding1 == cost1_binding2)
        self.assertEqual(cost1_binding1, cost1_binding2)

        # The bindings have the same variables but different expressions.
        self.assertNotEqual(cost1_binding1, cost2_binding)
        # The bindings have the same expression but different variables.
        self.assertNotEqual(cost1_binding1, cost3_binding)

    def test_binding_hash(self):
        x = sym.Variable("x")
        y = sym.Variable("y")
        e = np.log(2 * x) + y**2
        e2 = y

        cost1 = mp.ExpressionCost(e=e)
        cost1_binding1 = mp.Binding[mp.ExpressionCost](cost1, cost1.vars())
        cost1_binding2 = mp.Binding[mp.ExpressionCost](cost1, cost1.vars())

        cost2 = mp.ExpressionCost(e=e2)
        cost2_binding = mp.Binding[mp.ExpressionCost](cost2, cost2.vars())

        self.assertEqual(hash(cost1_binding1), hash(cost1_binding2))
        self.assertNotEqual(hash(cost1_binding1), hash(cost2_binding))

    def test_is_thread_safe(self):
        x = sym.Variable("x")
        y = sym.Variable("y")
        e1 = np.sin(x) + y
        cost1 = mp.ExpressionCost(e=e1)
        self.assertFalse(cost1.is_thread_safe())


class TestConstraints(unittest.TestCase):
    def test_bounding_box_constraint(self):
        constraint = mp.BoundingBoxConstraint(
            lb=np.array([1.0, 2.0]), ub=np.array([2.0, 3.0])
        )
        np.testing.assert_array_equal(
            constraint.lower_bound(), np.array([1.0, 2.0])
        )
        np.testing.assert_array_equal(
            constraint.upper_bound(), np.array([2.0, 3.0])
        )

    def test_linear_constraint(self):
        A_sparse = scipy.sparse.csc_matrix(
            (np.array([2, 1, 3]), np.array([0, 1, 0]), np.array([0, 2, 2, 3])),
            shape=(2, 2),
        )
        lb = -np.ones(2)
        ub = np.ones(2)

        constraints = []
        constraints.append(mp.LinearConstraint(A=np.eye(2), lb=lb, ub=ub))
        self.assertTrue(constraints[-1].is_dense_A_constructed())
        constraints.append(mp.LinearConstraint(A=A_sparse, lb=lb, ub=ub))
        self.assertFalse(constraints[-1].is_dense_A_constructed())

        for c in constraints:
            self.assertEqual(c.GetDenseA().shape[1], 2)
            self.assertEqual(c.get_sparse_A().shape[1], 2)
            r = c.GetDenseA().shape[0]
            new_A = np.ones_like(c.GetDenseA())
            new_A[1, 1] = 1e-20

            c.UpdateCoefficients(
                new_A=new_A, new_lb=np.ones(r), new_ub=np.ones(r)
            )
            c.RemoveTinyCoefficient(tol=1e-10)
            self.assertEqual(c.GetDenseA()[1, 1], 0)
            c.UpdateCoefficients(
                new_A=c.get_sparse_A(), new_lb=np.ones(r), new_ub=np.ones(r)
            )

            c.UpdateLowerBound(new_lb=-2 * np.ones(r))
            c.UpdateUpperBound(new_ub=2 * np.ones(r))
            c.set_bounds(new_lb=-3 * np.ones(r), new_ub=3 * np.ones(r))

    def test_quadratic_constraint(self):
        hessian_type = mp.QuadraticConstraint.HessianType.kPositiveSemidefinite
        constraint = mp.QuadraticConstraint(
            Q0=np.eye(2),
            b=np.array([1, 2.0]),
            lb=-np.inf,
            ub=1.0,
            hessian_type=hessian_type,
        )
        np.testing.assert_array_equal(constraint.Q(), np.eye(2))
        np.testing.assert_array_equal(constraint.b(), np.array([1, 2.0]))
        self.assertEqual(constraint.hessian_type(), hessian_type)
        self.assertTrue(constraint.is_convex())
        hessian_type = mp.QuadraticConstraint.HessianType.kNegativeSemidefinite
        constraint.UpdateCoefficients(
            new_Q=-np.eye(2),
            new_b=np.array([1.0, -1.0]),
            hessian_type=hessian_type,
        )
        self.assertEqual(constraint.hessian_type(), hessian_type)
        self.assertFalse(constraint.is_convex())
        constraint.UpdateCoefficients(
            new_Q=np.array([[1, 0], [0, -1.0]]), new_b=np.array([1.0, -1])
        )
        hessian_type = mp.QuadraticConstraint.HessianType.kIndefinite
        self.assertEqual(constraint.hessian_type(), hessian_type)

    def test_positive_semidefinite_constraint(self):
        constraint = mp.PositiveSemidefiniteConstraint(rows=3)
        self.assertEqual(constraint.matrix_rows(), 3)

    def test_linear_matrix_inequality_constraint(self):
        constraint = mp.LinearMatrixInequalityConstraint(
            F=[np.eye(3), 2 * np.eye(3), np.ones((3, 3))],
            symmetry_tolerance=1e-12,
        )
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
            sym.Variables(constraint.vars()), sym.Variables([x, y])
        )
        np.testing.assert_array_equal(lb, constraint.lower_bound())
        np.testing.assert_array_equal(ub, constraint.upper_bound())

    def test_lorentz_cone_constraint(self):
        A = np.array([[1, 2], [-1, -3], [2, 3.0]])
        b = np.array([2.0, 3.0, 4.0])
        constraint = mp.LorentzConeConstraint(
            A=A, b=b, eval_type=mp.LorentzConeConstraint.EvalType.kConvexSmooth
        )
        np.testing.assert_array_equal(constraint.A().todense(), A)
        np.testing.assert_array_equal(constraint.b(), b)
        self.assertEqual(
            constraint.eval_type(),
            mp.LorentzConeConstraint.EvalType.kConvexSmooth,
        )
        constraint.UpdateCoefficients(new_A=2 * A, new_b=3 * b)
        np.testing.assert_array_equal(constraint.A().todense(), 2 * A)
        np.testing.assert_array_equal(constraint.b(), 3 * b)

    def test_rotated_lorentz_cone_constraint(self):
        A = np.array(
            [
                [1.0, 2.0, 3.0],
                [4.0, 5.0, 6.0],
                [7.0, 8.0, 9.0],
                [10.0, 11.0, 12.0],
            ]
        )
        b = np.array([1.0, 2.0, 3, 4])
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
            mp.MinimumValueLowerBoundConstraint,
            mp.MinimumValueUpperBoundConstraint,
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

    def test_binding_cast_python_wrapper_lost(self):
        # Ensure constraints' python wrappers are kept alive when passed
        # through the casting constructor. See issue #20131 for original
        # problem description.

        # Make some objects in a function to let most of them be deleted at
        # scope exit. We will test if the contents of the first return value
        # ("keepers") succeed in keeping alive the objects tracked by "spies".
        def make_object_graph():
            spies = []
            x = sym.Variable("x")
            y = sym.Variable("y")
            e = np.sin(x) + y
            constraint = mp.ExpressionConstraint(
                v=np.array([e]), lb=np.array([1.0]), ub=np.array([2.0])
            )
            # Sett up the types to ensure the cast constructor is used.
            constraint_binding = mp.Binding[mp.ExpressionConstraint](
                constraint, constraint.vars()
            )
            cast_binding = mp_testing.AcceptBindingEvaluatorBase(
                constraint_binding
            )
            spies.append(weakref.finalize(constraint, lambda: None))
            return cast_binding, spies

        keeper, spies = make_object_graph()
        self.assertTrue(all(spy.alive for spy in spies))

    def test_binding_constraint_python_wrapper_lost(self):
        # Ensure constraints' python wrappers are kept alive when bound. See
        # issue #20131 for original problem description.

        # Make some objects in a function to let most of them be deleted at
        # scope exit. We will test if the contents of the first return value
        # ("keepers") succeed in keeping alive the objects tracked by "spies".
        def make_object_graph():
            spies = []
            x = sym.Variable("x")
            y = sym.Variable("y")
            e = np.sin(x) + y
            constraint = mp.ExpressionConstraint(
                v=np.array([e]), lb=np.array([1.0]), ub=np.array([2.0])
            )
            constraint_binding = mp.Binding[mp.ExpressionConstraint](
                constraint, constraint.vars()
            )
            spies.append(weakref.finalize(constraint, lambda: None))
            return constraint_binding, spies

        keeper, spies = make_object_graph()
        self.assertTrue(all(spy.alive for spy in spies))

    def test_binding_eq(self):
        x = sym.Variable("x")
        y = sym.Variable("y")
        z = sym.Variable("z")
        e1 = np.sin(x) + y
        e2 = np.cos(x) + y
        e3 = np.sin(z) + y
        constraint1 = mp.ExpressionConstraint(
            v=np.array([e1]), lb=np.array([1.0]), ub=np.array([2.0])
        )
        constraint1_binding1 = mp.Binding[mp.ExpressionConstraint](
            constraint1, constraint1.vars()
        )
        constraint1_binding2 = mp.Binding[mp.ExpressionConstraint](
            constraint1, constraint1.vars()
        )

        constraint2 = mp.ExpressionConstraint(
            v=np.array([e2]), lb=np.array([1.0]), ub=np.array([2.0])
        )
        constraint2_binding = mp.Binding[mp.ExpressionConstraint](
            constraint2, constraint2.vars()
        )

        constraint3 = mp.ExpressionConstraint(
            v=np.array([e3]), lb=np.array([1.0]), ub=np.array([2.0])
        )
        constraint3_binding = mp.Binding[mp.ExpressionConstraint](
            constraint3, constraint3.vars()
        )

        self.assertTrue(constraint1_binding1 == constraint1_binding1)
        self.assertTrue(constraint1_binding1 == constraint1_binding2)
        self.assertEqual(constraint1_binding1, constraint1_binding2)

        # The bindings have the same variables but different expressions.
        self.assertNotEqual(constraint1_binding1, constraint2_binding)
        # The bindings have the same expression but different variables.
        self.assertNotEqual(constraint1_binding1, constraint3_binding)

    def test_binding_hash(self):
        x = sym.Variable("x")
        y = sym.Variable("y")
        e = np.log(2 * x) + y**2
        e2 = y
        constraint1 = mp.ExpressionConstraint(
            v=np.array([e]), lb=np.array([1.0]), ub=np.array([2.0])
        )
        constraint1_binding1 = mp.Binding[mp.ExpressionConstraint](
            constraint1, constraint1.vars()
        )
        constraint1_binding2 = mp.Binding[mp.ExpressionConstraint](
            constraint1, constraint1.vars()
        )
        constraint2 = mp.ExpressionConstraint(
            v=np.array([e2]), lb=np.array([1.0]), ub=np.array([2.0])
        )
        self.assertEqual(hash(constraint1_binding1), hash(constraint1_binding2))
        self.assertNotEqual(hash(constraint1_binding1), hash(constraint2))


# A dummy value function for MinimumValue{Lower,Upper}BoundConstraint.
def value_function(x: np.ndarray, v_influence: float) -> np.ndarray:
    x0 = x[0]
    return np.array([x0**2, x0 + 1, 2 * x0])


class TestMinimumValueLowerBoundConstraint(unittest.TestCase):
    def test_constructor(self):
        # Test the constructor with value_function_double explicitly specified.
        dut = mp.MinimumValueLowerBoundConstraint(
            num_vars=1,
            minimum_value_lower=0.0,
            influence_value_offset=1.0,
            max_num_values=3,
            value_function=value_function,
            value_function_double=value_function,
        )
        self.assertEqual(dut.num_vars(), 1)
        self.assertEqual(dut.num_constraints(), 1)
        self.assertTrue(dut.CheckSatisfied(np.array([1.0])))
        self.assertFalse(dut.CheckSatisfied(np.array([-5.0])))
        self.assertEqual(dut.minimum_value_lower(), 0.0)
        self.assertEqual(dut.influence_value(), 1.0)

        # Test the constructor with default value_function_double.
        dut = mp.MinimumValueLowerBoundConstraint(
            num_vars=1,
            minimum_value_lower=0.0,
            influence_value_offset=1.0,
            max_num_values=3,
            value_function=value_function,
        )
        self.assertTrue(dut.CheckSatisfied(np.array([1.0])))
        self.assertFalse(dut.CheckSatisfied(np.array([-5.0])))

    def test_set_penalty_function(self):
        dut = mp.MinimumValueLowerBoundConstraint(
            num_vars=1,
            minimum_value_lower=0.0,
            influence_value_offset=3.0,
            max_num_values=3,
            value_function=value_function,
            value_function_double=value_function,
        )

        # Now set the new penalty function
        def penalty(
            x: float, compute_grad: bool
        ) -> typing.Tuple[float, typing.Optional[float]]:
            if x < 0:
                if compute_grad:
                    return x**2, 2 * x
                else:
                    return x**2, None
            else:
                if compute_grad:
                    return 0.0, 0.0
                else:
                    return 0.0, None

        dut.set_penalty_function(new_penalty_function=penalty)
        self.assertTrue(dut.CheckSatisfied(np.array([1.0])))
        self.assertFalse(dut.CheckSatisfied(np.array([-5.0])))
        self.assertTrue(dut.CheckSatisfied(InitializeAutoDiff(np.array([1.0]))))
        self.assertFalse(dut.CheckSatisfied(InitializeAutoDiff(np.array([-2]))))


class TestMinimumValueUpperBoundConstraint(unittest.TestCase):
    def test_constructor(self):
        # Test the constructor with value_function_double explicitly specified.
        dut = mp.MinimumValueUpperBoundConstraint(
            num_vars=1,
            minimum_value_upper=2.0,
            influence_value_offset=1.0,
            max_num_values=3,
            value_function=value_function,
            value_function_double=value_function,
        )
        self.assertEqual(dut.num_vars(), 1)
        self.assertEqual(dut.num_constraints(), 1)
        self.assertTrue(dut.CheckSatisfied(np.array([1.0])))
        self.assertFalse(dut.CheckSatisfied(np.array([5.0])))
        self.assertEqual(dut.minimum_value_upper(), 2)
        self.assertEqual(dut.influence_value(), 3)

        # Test the constructor with default value_function_double.
        dut = mp.MinimumValueUpperBoundConstraint(
            num_vars=1,
            minimum_value_upper=2.0,
            influence_value_offset=1.0,
            max_num_values=3,
            value_function=value_function,
        )
        self.assertTrue(dut.CheckSatisfied(np.array([1.0])))
        self.assertFalse(dut.CheckSatisfied(np.array([5.0])))

    def test_set_penalty_function(self):
        dut = mp.MinimumValueUpperBoundConstraint(
            num_vars=1,
            minimum_value_upper=1.5,
            influence_value_offset=3.0,
            max_num_values=3,
            value_function=value_function,
            value_function_double=value_function,
        )

        # Now set the new penalty function
        def penalty(
            x: float, compute_grad: bool
        ) -> typing.Tuple[float, typing.Optional[float]]:
            if x < 0:
                if compute_grad:
                    return x**2, 2 * x
                else:
                    return x**2, None
            else:
                if compute_grad:
                    return 0.0, 0.0
                else:
                    return 0.0, None

        dut.set_penalty_function(new_penalty_function=penalty)
        self.assertTrue(dut.CheckSatisfied(np.array([1.0])))
        self.assertTrue(dut.CheckSatisfied(np.array([-5.0])))
        self.assertTrue(dut.CheckSatisfied(InitializeAutoDiff(np.array([1.0]))))
        self.assertFalse(
            dut.CheckSatisfied(InitializeAutoDiff(np.array([5.0])))
        )
