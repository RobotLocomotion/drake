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
            # Boolean functions, ordered to match FormulaKind.
            x == y,
            x != y,
            x > y,
            x >= y,
            x < y,
            x <= y,
            # TODO(jwnimmer-tri) Fix me.
            # mut.logical_and(q, r),
            # mut.logical_or(q, r),
            # mut.logical_not(q),
        ]
        for item in inputs:
            with self.subTest(item=item):
                memo = dict()
                converted = mut.to_sympy(item, memo=memo)
                readback = mut.from_sympy(converted, memo=memo)
                if isinstance(item, (float, bool)):
                    np.testing.assert_equal(item, readback)
                else:
                    self.assertTrue(item.EqualTo(readback), msg=repr(readback))
