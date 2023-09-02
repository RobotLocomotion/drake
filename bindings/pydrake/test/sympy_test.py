import pydrake.symbolic as mut

import math
import numpy as np
import unittest

import sympy

from pydrake.symbolic import (
    Expression,
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
            1.0,
            math.nan,
            Expression(1.0),
            Expression(math.nan),
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

    def _test_one_round_trip(self, item):
        memo = dict()
        converted = mut.to_sympy(item, memo=memo)
        readback = mut.from_sympy(converted, memo=memo)

        # When the input was a built-in type, np.testing works well.
        if isinstance(item, (float, bool)):
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
            pass
        self.fail(f"Incomparable {item_str} with "
                  f"{readback_str} (from {converted_str})")
