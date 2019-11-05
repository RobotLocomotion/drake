# -*- coding: utf-8 -*-

import unittest
import numpy as np
import pydrake.symbolic as sym

# Define global variables to make the tests less verbose.
x = sym.Variable("x")
y = sym.Variable("y")
z = sym.Variable("z")
w = sym.Variable("w")


class TestSymbolicVariables(unittest.TestCase):
    def test_default_constructor(self):
        vars = sym.Variables()
        self.assertEqual(vars.size(), 0)
        self.assertTrue(vars.empty())

    def test_constructor_list(self):
        vars = sym.Variables([x, y, z])
        self.assertEqual(vars.size(), 3)

    def test_insert1(self):
        vars = sym.Variables()
        vars.insert(x)
        self.assertEqual(vars.size(), 1)

    def test_insert2(self):
        vars = sym.Variables([x])
        vars.insert(sym.Variables([y, z]))
        self.assertEqual(vars.size(), 3)

    def test_erase1(self):
        vars = sym.Variables([x, y, z])
        count = vars.erase(x)
        self.assertEqual(count, 1)

    def test_erase2(self):
        vars1 = sym.Variables([x, y, z])
        vars2 = sym.Variables([w, z])
        count = vars1.erase(vars2)
        self.assertEqual(count, 1)
        self.assertEqual(vars1.size(), 2)

    def test_include(self):
        vars = sym.Variables([x, y, z])
        self.assertTrue(vars.include(y))

    def test_to_string(self):
        vars = sym.Variables()
        vars.insert(x)
        vars.insert(y)
        vars.insert(z)
        self.assertEqual(vars.to_string(), "{x, y, z}")

    def test_subset_properties(self):
        vars1 = sym.Variables([x, y, z])
        vars2 = sym.Variables([x, y])
        self.assertFalse(vars1.IsSubsetOf(vars2))
        self.assertFalse(vars1.IsStrictSubsetOf(vars2))
        self.assertTrue(vars1.IsSupersetOf(vars2))
        self.assertTrue(vars1.IsStrictSupersetOf(vars2))

    def test_eq(self):
        vars1 = sym.Variables([x, y, z])
        vars2 = sym.Variables([x, y])
        self.assertFalse(vars1 == vars2)

    def test_lt(self):
        vars1 = sym.Variables([x, y])
        vars2 = sym.Variables([x, y, z])
        self.assertTrue(vars1 < vars2)

    def test_add(self):
        vars1 = sym.Variables([x, y])
        vars2 = sym.Variables([y, z])
        vars3 = vars1 + vars2  # [x, y, z]
        self.assertEqual(vars3.size(), 3)
        vars4 = vars1 + z  # [x, y, z]
        self.assertEqual(vars4.size(), 3)
        vars5 = x + vars1  # [x, y]
        self.assertEqual(vars5.size(), 2)

    def test_add_assignment(self):
        vars = sym.Variables([x])
        vars += y
        self.assertEqual(vars.size(), 2)
        vars += sym.Variables([x, z])
        self.assertEqual(vars.size(), 3)

    def test_sub(self):
        vars1 = sym.Variables([x, y])
        vars2 = sym.Variables([y, z])
        vars3 = vars1 - vars2  # [x]
        self.assertEqual(vars3, sym.Variables([x]))
        vars4 = vars1 - y  # [x]
        self.assertEqual(vars4, sym.Variables([x]))

    def test_sub_assignment(self):
        vars = sym.Variables([x, y, z])
        vars -= y  # = [x, z]
        self.assertEqual(vars, sym.Variables([x, z]))
        vars -= sym.Variables([x])  # = [z]
        self.assertEqual(vars, sym.Variables([z]))

    def test_intersect(self):
        vars1 = sym.Variables([x, y, z])
        vars2 = sym.Variables([y, w])
        vars3 = sym.intersect(vars1, vars2)  # = [y]
        self.assertEqual(vars3, sym.Variables([y]))
