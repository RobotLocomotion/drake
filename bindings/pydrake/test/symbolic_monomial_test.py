# -*- coding: utf-8 -*-

import unittest
import numpy as np
import pydrake.symbolic as sym

# Define global variables to make the tests less verbose.
x = sym.Variable("x")
y = sym.Variable("y")
z = sym.Variable("z")


class TestSymbolicMonomial(unittest.TestCase):
    def test_constructor_variable(self):
        m = sym.Monomial(x)  # m = x¹
        self.assertEqual(m.degree(x), 1)
        self.assertEqual(m.total_degree(), 1)

    def test_constructor_variable_int(self):
        m = sym.Monomial(x, 2)  # m = x²
        self.assertEqual(m.degree(x), 2)
        self.assertEqual(m.total_degree(), 2)

    def test_constructor_map(self):
        powers_in = {x: 2, y: 3, z: 4}
        m = sym.Monomial(powers_in)
        powers_out = m.get_powers()
        self.assertEqual(powers_out[x], 2)
        self.assertEqual(powers_out[y], 3)
        self.assertEqual(powers_out[z], 4)

    def test_comparison(self):
        # m1 = m2 = x²
        m1 = sym.Monomial(x, 2)
        m2 = sym.Monomial(x, 2)
        m3 = sym.Monomial(x, 1)
        m4 = sym.Monomial(y, 2)
        # Test operator==
        self.assertTrue(m1 == m2)
        self.assertFalse(m1 == m3)
        self.assertFalse(m1 == m4)
        self.assertFalse(m2 == m3)
        self.assertFalse(m2 == m4)
        self.assertFalse(m3 == m4)
        # Test operator!=
        self.assertFalse(m1 != m2)
        self.assertTrue(m1 != m3)
        self.assertTrue(m1 != m4)
        self.assertTrue(m2 != m3)
        self.assertTrue(m2 != m4)
        self.assertTrue(m3 != m4)

    def test_str(self):
        m1 = sym.Monomial(x, 2)
        self.assertEqual(str(m1), "x^2")
        m2 = m1 * y
        self.assertEqual(str(m2), "x^2 * y")

    def test_multiplication1(self):
        m1 = sym.Monomial(x, 2)
        m2 = sym.Monomial(y, 3)
        m3 = m1 * m2
        self.assertEqual(m3.degree(x), 2)
        self.assertEqual(m3.degree(y), 3)

    def test_multiplication2(self):
        m1 = sym.Monomial(x, 2)
        m2 = m1 * y
        m3 = y * m1
        self.assertEqual(m2.degree(x), 2)
        self.assertEqual(m2.degree(y), 1)
        self.assertEqual(m2, m3)

    def test_multiplication_assignment1(self):
        m = sym.Monomial(x, 2)
        m *= sym.Monomial(y, 3)
        self.assertEqual(m.degree(x), 2)
        self.assertEqual(m.degree(y), 3)

    def test_multiplication_assignment2(self):
        m = sym.Monomial(x, 2)
        m *= y
        self.assertEqual(m.degree(x), 2)
        self.assertEqual(m.degree(y), 1)

    def test_pow(self):
        m1 = sym.Monomial(x, 2) * y  # m1 = x²y
        m2 = m1 ** 2                 # m2 = x⁴y²
        self.assertEqual(m2.degree(x), 4)
        self.assertEqual(m2.degree(y), 2)

    def test_pow_in_place(self):
        m1 = sym.Monomial(x, 2) * y  # m1 = x²y
        m2 = m1.pow_in_place(2)      # m1 = m2 = x⁴y²
        self.assertEqual(m1.degree(x), 4)
        self.assertEqual(m1.degree(y), 2)
        self.assertEqual(m2.degree(x), 4)
        self.assertEqual(m2.degree(y), 2)

    def test_get_powers(self):
        m = sym.Monomial(x, 2) * y  # m = x²y
        powers = m.get_powers()
        self.assertEqual(powers[x], 2)
        self.assertEqual(powers[y], 1)

    def test_to_expression(self):
        m = sym.Monomial(x, 3) * y  # m = x³y
        e = m.ToExpression()
        self.assertEqual(str(e), "(pow(x, 3) * y)")

    def test_get_variables(self):
        m = sym.Monomial(x, 3) * y  # m = x³y
        vars = m.GetVariables()  # = [x, y]
        self.assertEqual(vars.size(), 2)

    def test_monomial_basis(self):
        vars = sym.Variables([x, y, z])
        basis1 = sym.MonomialBasis(vars, 3)
        basis2 = sym.MonomialBasis([x, y, z], 3)
        self.assertEqual(basis1.size, 20)
        self.assertEqual(basis2.size, 20)
