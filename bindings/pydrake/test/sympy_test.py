import pydrake.symbolic as mut

import math
import numpy as np
import unittest

import sympy

from pydrake.symbolic import (
    Expression,
    Variable,
)


class TestSympy(unittest.TestCase):

    def test_round_trip(self):
        x, y, z = Variable("x"), Variable("y"), Variable("z")
        expressions = [
            # Constants.
            1.0,
            Expression(1.0),
            math.nan,
            # Variables.
            x,
            Expression(x),
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
        ]
        for e in expressions:
            with self.subTest(e=e):
                memo = dict()
                converted = mut.to_sympy(e, memo=memo)
                readback = mut.from_sympy(converted, memo=memo)
                if isinstance(e, float):
                    np.testing.assert_equal(e, readback)
                else:
                    self.assertTrue(e.EqualTo(readback), msg=repr(readback))
