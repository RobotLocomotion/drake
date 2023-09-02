import pydrake.symbolic as mut

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
            Expression(1.0),
            x,
            Expression(x),
            x + y,
            y * z,
            x + y * z,
        ]
        for e in expressions:
            with self.subTest(e=e):
                memo = dict()
                converted = mut.to_sympy(e, memo=memo)
                readback = mut.from_sympy(converted, memo=memo)
                self.assertTrue(e.EqualTo(readback), msg=repr(readback))
