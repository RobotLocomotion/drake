"""
Provides utilities to check if an object supports pickling (serlialization).
"""

from collections import Counter
import pickle

import numpy as np

import pydrake.common.test_utilities.numpy_compare as numpy_compare
from pydrake.symbolic import (
    Expression,
    ExpressionKind,
)

_PYBIND11_METACLASS = type(Expression)


def _assert_equal(test, a, b):
    if isinstance(a, np.ndarray):
        numpy_compare.assert_equal(a, b)
    else:
        test.assertEqual(a, b)


def _pickle_and_unpickle(obj):
    return pickle.loads(pickle.dumps(obj))


def _count_var(vars: set) -> Counter:
    return Counter(var.get_name() for var in vars)


def _sort_vars(vars: set) -> list:
    return sorted(vars, key=lambda v: v.get_name())


def assert_pickle_expression(test, expr: Expression, evaluate: bool):
    """
    Asserts structural equality of expressions before and after pickling
    by checking that the unpickled expression maintains:
    1) same number of variables
    2) same variable names
    3) evaluation value given the same environment.
    Args:
        test: test case
        expr: expression to test
        evaluate: flag to evaulate expression undert test
    """
    expr_again = _pickle_and_unpickle(expr)
    vars = expr.GetVariables()
    vars_again = expr_again.GetVariables()
    test.assertEqual(len(vars), len(vars_again))
    vars_count = _count_var(vars)
    vars_again_count = _count_var(vars_again)
    test.assertEqual(vars_count, vars_again_count)
    if expr.get_kind() in (ExpressionKind.Constant, ExpressionKind.NaN):
        test.assertTrue(expr.EqualTo(expr_again))
        return
    if evaluate is False:
        return
    """
    Names of variables must be unique in expressions to generalise the 
    evaluation algorithm. The following is a counterexample:
    Suppose vars = {x=Variable("x"), y=Variable("x")}, 
    and vars_again = {y=Variable("x"), x=Variable("x")}.
    It follows that sorted_vars_list = [x=Variable("x"), y=Variable("x")], 
    and sorted_vars_again_list = [y=Variable("x"), x=Variable("x")].
    The two `Variables` objects have different order. Thus, 
    env_vars = {x:1, y:2}, and env_vars_again = {y:1, x:2}.
    If the before and after expressions are equal, then the different 
    environments might lead to non-equal evaluations.
    """
    sorted_vars_list = _sort_vars(vars)
    sorted_vars_again_list = _sort_vars(vars_again)
    env_vars = {v: i for i, v in enumerate(sorted_vars_list, start=1)}
    env_vars_again = {
        v: i for i, v in enumerate(sorted_vars_again_list, start=1)
    }
    test.assertEqual(
        expr.Evaluate(env_vars), expr_again.Evaluate(env_vars_again)
    )


def _assert_expression_matrices_equal(test, mat, mat_again):
    test.assertTrue(len(mat) == len(mat_again))
    test.assertTrue(len(mat[0]) == len(mat_again[0]))
    for i in range(len(mat)):
        for j in range(len(mat[0])):
            expr = mat[i][j]
            expr_again = mat_again[i][j]
            if (
                expr.get_kind()
                == expr_again.get_kind()
                == ExpressionKind.Constant
            ):
                test.assertTrue(expr.EqualTo(expr_again))
            else:
                test.fail(
                    "equality of non constant expressions in not implemented"
                )


def _assert_expression_vectors_equal(test, vector, vector_again):
    test.assertEqual(len(vector), len(vector_again))
    for expr, expr_again in zip(vector, vector_again):
        if expr.get_kind() == expr_again.get_kind() == ExpressionKind.Constant:
            test.assertTrue(expr.EqualTo(expr_again))
        else:
            test.fail("equality of non constant expressions in not implemented")


def assert_pickle(test, obj, value_to_compare=lambda x: x.__dict__, T=None):
    """
    Asserts that an object can be dumped and loaded and still maintain its
    value.

    Args:
        test: Instance of `unittest.TestCase` (for assertions).
        obj: Obj to dump and then load.
        value_to_compare: (optional) Value to extract from the object to
            compare. By default, compares dictionaries.
        T: (optional) When pickling template instantiations on scalar types,
            pass the scalar type T.
    """
    from pydrake.math import (
        BsplineBasis_,
        RigidTransform_,
        RollPitchYaw_,
        RotationMatrix_,
    )

    obj_again = _pickle_and_unpickle(obj)
    if T == Expression:
        if isinstance(obj, RotationMatrix_[Expression]):
            _assert_expression_matrices_equal(
                test, obj.matrix(), obj_again.matrix()
            )
        elif isinstance(obj, RigidTransform_[Expression]):
            _assert_expression_matrices_equal(
                test, obj.rotation().matrix(), obj_again.rotation().matrix()
            )
            _assert_expression_vectors_equal(
                test, obj.translation(), obj_again.translation()
            )
        elif isinstance(obj, RollPitchYaw_[Expression]):
            _assert_expression_vectors_equal(test, obj.vector(), obj.vector())
        elif isinstance(obj, BsplineBasis_[Expression]):
            _assert_expression_vectors_equal(test, obj.knots(), obj.knots())
    else:
        _assert_equal(test, value_to_compare(obj), value_to_compare(obj_again))
