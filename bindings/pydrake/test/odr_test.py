"""
This tests ODR for how we wrap our Python.
This is tested explicitly with symbolic::Variable, given that it has global
state, and ensures that we have unique symbols generated from two different
modules in sequence. If we fail this, then that means we have multiple
definitions of `drake::symbolic::Variable::get_next_id()`.

@see #6465
"""


import unittest

import pydrake.symbolic as sym
# This module also uses `//common:symbolic`, but in a separate *.so.
# If we violate ODR, then this might link in a new definition.
from pydrake.test import odr_test_module


class TestODR(unittest.TestCase):
    def test_variable(self):
        x1 = sym.Variable('x')
        x2 = odr_test_module.new_variable('x')
        self.assertNotEqual(x1.get_id(), x2.get_id())
