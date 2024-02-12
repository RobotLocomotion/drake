import pydrake.symbolic as mut

import numpy as np
import unittest

import sympy

from pydrake.symbolic import (
    Expression,
    ExpressionKind,
    Formula,
    Variable,
)

BOOLEAN = Variable.Type.BOOLEAN


class TestSympy(unittest.TestCase):

    def test_round_trip(self):
        x, y, z = [Variable(name) for name in "x y z".split()]
        q, r = [Variable(name, BOOLEAN) for name in "q r".split()]
        inputs = [
            # Constants.
            1,
            1.0,
            np.nan,
            Expression(1.0),
            Expression(np.nan),
            True,
            False,
            Formula(True),
            Formula(False),
            # Variables.
            x,
            Expression(x),
            q,
            Formula(q),
            # Arithmetic.
            x + y,
            y * z,
            x + y * z,
            x - y,
            x / y,
            x - y / z,
            x ** y,
            # Unary and binary functions, ordered to match ExpressionKind.
            mut.log(x),
            mut.abs(x),
            mut.exp(x),
            mut.sqrt(x),
            mut.pow(x, y),
            mut.sin(x),
            mut.cos(x),
            mut.tan(x),
            mut.asin(x),
            mut.acos(x),
            mut.atan(x),
            mut.atan2(x, y),
            mut.sinh(x),
            mut.cosh(x),
            mut.tanh(x),
            mut.min(y, x),
            mut.max(y, x),
            mut.ceil(x),
            mut.floor(x),
            mut.if_then_else(q, x, y),
            # Boolean functions, ordered to match FormulaKind.
            # First with op(x, y).
            x == y,
            x != y,
            x > y,
            x >= y,
            x < y,
            x <= y,
            # Next with op(y, x).
            y == x,
            y != x,
            y > x,
            y >= x,
            y < x,
            y <= x,
            # Next the module functions.
            mut.logical_and(q, r),
            mut.logical_or(q, r),
            mut.logical_and(r, q),
            mut.logical_or(r, q),
            mut.logical_not(q),
            # Combine and nest multiple kinds of functions.
            mut.if_then_else(y > x, x + mut.pow(y, 3), mut.exp(z)),
        ]
        for item in inputs:
            with self.subTest(item=item):
                self._test_one_round_trip(item)
                self._test_one_eval(item)

    def _test_one_round_trip(self, item):
        memo = dict()
        converted = mut.to_sympy(item, memo=memo)
        readback = mut.from_sympy(converted, memo=memo)

        # When the input was a built-in type, np.testing works well.
        if isinstance(item, (float, int, bool)):
            np.testing.assert_equal(item, readback)
            return

        # When the input was a Drake type, check for structral equality.
        item_str = f"item={item!r} ({type(item)})"
        converted_str = f"converted={converted!r} ({type(converted)})"
        readback_str = f"readback={readback!r} ({type(readback)})"
        try:
            self.assertTrue(item.EqualTo(readback),
                            msg=f"Got {readback_str} (from {converted_str})")
            return
        except TypeError as e:
            # We'll do a self.fail(), but outside the except. Otherwise we are
            # bombarded by Python's "a new exception happened in the middle of
            # handling a prior exception" vomit that isn't useful in this case.
            pass
        self.fail(f"Incomparable {item_str} with "
                  f"{readback_str} (from {converted_str})")

    def _test_one_eval(self, item):
        if isinstance(item, (float, int, bool, Variable)):
            # Skip this case. Evaluation is not particularly interesting.
            return

        if isinstance(item, Formula):
            # TODO(jwnimmer-tri) Figure out how to eval a SymPy formula. The
            # naive evalf with bools in `subs` does not work.
            return

        assert isinstance(item, Expression), repr(item)
        if item.get_kind() == ExpressionKind.IfThenElse:
            # TODO(jwnimmer-tri) Figure out how to eval a SymPy formula. The
            # naive evalf with bools in `subs` does not work.
            return

        # Create a handy dictionary of Variable Id => Variable.
        drake_vars = dict([
            (var.get_id(), var)
            for var in item.GetVariables()
        ])
        if not drake_vars:
            # Skip this case. Evaluation is not particularly interesting.
            return

        # Evaluate the `item` using Drake's evaluation function.
        drake_env = dict()
        for i, var in enumerate(drake_vars.values()):
            assert var.get_type() == Variable.Type.CONTINUOUS
            drake_env[var] = (i + 1) * 0.1
        drake_value = item.Evaluate(drake_env)

        # Convert to SymPy.
        memo = dict()
        sympy_item = mut.to_sympy(item, memo=memo)

        # Evaluate the `sympy_item` using SymPy's evaluation function.
        sympy_env = dict()
        for drake_var_id, drake_var in drake_vars.items():
            sympy_var = memo[drake_var_id]
            sympy_env[sympy_var] = drake_env[drake_var]
        sympy_value = sympy_item.evalf(subs=sympy_env)

        # The two evaluations must match.
        self.assertAlmostEqual(drake_value, sympy_value)
