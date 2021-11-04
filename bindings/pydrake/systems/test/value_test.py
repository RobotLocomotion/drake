# -*- coding: utf-8 -*-

import copy
import unittest

import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.common.value import AbstractValue, Value
from pydrake.common.test_utilities import numpy_compare
from pydrake.symbolic import Expression
from pydrake.systems.framework import (
    BasicVector, BasicVector_,
    Parameters,
    VectorBase,
    )


def pass_through(x):
    return x

# TODO(eric.cousineau): Add negative (or positive) test cases for AutoDiffXd
# and Symbolic once they are in the bindings.


class TestValue(unittest.TestCase):
    def test_basic_vector_double(self):
        # Test constructing vectors of sizes [0, 1, 2], and ensure that we can
        # construct from both lists and `np.array` objects with no ambiguity.
        for n in [0, 1, 2]:
            for wrap in [pass_through, np.array]:
                # Ensure that we can get vectors templated on double by
                # reference.
                expected_init = wrap([float(x) for x in range(n)])
                expected_add = wrap([x + 1 for x in expected_init])
                expected_set = wrap([x + 10 for x in expected_init])
                expected_plus_eq = wrap([3*x for x in expected_set])

                value_data = BasicVector(expected_init)
                value = value_data.get_mutable_value()
                self.assertTrue(np.allclose(value, expected_init))

                # Add value directly.
                # TODO(eric.cousineau): Determine if there is a way to extract
                # the pointer referred to by the buffer (e.g. `value.data`).
                value[:] += 1
                self.assertTrue(np.allclose(value, expected_add))
                self.assertTrue(
                    np.allclose(value_data.value(), expected_add))
                self.assertTrue(
                    np.allclose(value_data.get_value(), expected_add))
                self.assertTrue(
                    np.allclose(value_data.get_mutable_value(), expected_add))

                # Set value from `BasicVector`.
                value_data.SetFromVector(value=expected_set)
                self.assertTrue(np.allclose(value, expected_set))
                self.assertTrue(
                    np.allclose(value_data.get_value(), expected_set))
                self.assertTrue(
                    np.allclose(value_data.get_mutable_value(), expected_set))

                # Set value to zero.
                old_value = value_data.CopyToVector()
                value_data.SetZero()
                if n > 0:
                    self.assertFalse(np.allclose(old_value, np.zeros(n)))
                self.assertTrue(np.allclose(value, np.zeros(n)))
                self.assertTrue(
                    np.allclose(value_data.get_value(), np.zeros(n)))
                self.assertTrue(
                    np.allclose(value_data.get_mutable_value(), np.zeros(n)))

                # Set value from `BasicVector`.
                value_data.set_value(expected_set)
                self.assertTrue(np.allclose(value, expected_set))
                self.assertTrue(
                    np.allclose(value_data.get_value(), expected_set))
                self.assertTrue(
                    np.allclose(value_data.get_mutable_value(), expected_set))

                # Ensure we can construct from size.
                old_value_data = value_data
                value_data = BasicVector(n)
                self.assertEqual(value_data.size(), n)
                value_data.SetFrom(value=old_value_data)
                self.assertTrue(
                    np.allclose(value_data.get_value(), expected_set))
                new_value_data = value_data.PlusEqScaled(scale=2,
                                                         rhs=old_value_data)
                self.assertTrue(
                    np.allclose(value_data.get_value(), expected_plus_eq))
                # Ensure we can clone.
                value_copies = [
                    value_data.Clone(),
                    copy.copy(value_data),
                    copy.deepcopy(value_data),
                ]
                for value_copy in value_copies:
                    self.assertTrue(value_copy is not value_data)
                    self.assertEqual(value_data.size(), n)

    def test_basic_vector_set_get(self):
        value = BasicVector(np.arange(3., 5.))
        self.assertEqual(value.GetAtIndex(index=1), 4.)
        value.SetAtIndex(index=1, value=5.)
        self.assertEqual(value[1], 5.)
        value[1] = 6.
        self.assertEqual(value[1], 6.)

    def assert_basic_vector_equal(self, a, b):
        self.assertIs(type(a), type(b))
        self.assertIsNot(a, b)
        np.testing.assert_equal(a.get_value(), b.get_value())

    def test_str_and_repr(self):
        # T=float
        self.assertIs(BasicVector, BasicVector_[float])
        vector_f = [1.]
        value_f = BasicVector_[float](vector_f)
        self.assertEqual(str(value_f), "[1.0]")
        self.assertEqual(repr(value_f), "BasicVector_[float]([1.0])")
        # Check repr() invariant.
        self.assert_basic_vector_equal(value_f, eval(repr(value_f)))
        # - Empty.
        value_f_empty = BasicVector_[float]([])
        self.assertEqual(str(value_f_empty), "[]")
        self.assertEqual(repr(value_f_empty), "BasicVector_[float]([])")
        # - Multiple values.
        value_f_multi = BasicVector_[float]([1., 2.])
        self.assertEqual(str(value_f_multi), "[1.0, 2.0]")
        self.assertEqual(
            repr(value_f_multi), "BasicVector_[float]([1.0, 2.0])")
        # TODO(eric.cousineau): Make repr() for AutoDiffXd and Expression be
        # semi-usable.
        # T=AutoDiffXd
        value_ad = BasicVector_[AutoDiffXd](vector_f)
        self.assertEqual(str(value_ad), "[<AutoDiffXd 1.0 nderiv=0>]")
        self.assertEqual(
            repr(value_ad),
            "BasicVector_[AutoDiffXd]([<AutoDiffXd 1.0 nderiv=0>])")
        # T=Expression
        value_sym = BasicVector_[Expression](vector_f)
        self.assertEqual(str(value_sym), "[<Expression \"1\">]")
        self.assertEqual(
            repr(value_sym),
            "BasicVector_[Expression]([<Expression \"1\">])")

    @numpy_compare.check_all_types
    def test_value_registration(self, T):
        Value[BasicVector_[T]]

    def test_parameters_api(self):

        def compare(actual, expected):
            self.assertEqual(type(actual), type(expected))
            if isinstance(actual, VectorBase):
                self.assertTrue(
                    np.allclose(actual.get_value(), expected.get_value()))
            else:
                self.assertEqual(actual.get_value(), expected.get_value())

        model_numeric = BasicVector([0.])
        model_abstract = AbstractValue.Make("Hello")

        params = Parameters(
            numeric=[model_numeric.Clone()], abstract=[model_abstract.Clone()])
        self.assertEqual(params.num_numeric_parameter_groups(), 1)
        self.assertEqual(params.num_abstract_parameters(), 1)
        # Numeric.
        compare(params.get_numeric_parameter(index=0), model_numeric)
        compare(params.get_mutable_numeric_parameter(index=0), model_numeric)
        # WARNING: This will invalidate old references!
        params.set_numeric_parameters(params.get_numeric_parameters().Clone())
        # Abstract.
        compare(params.get_abstract_parameter(index=0), model_abstract)
        compare(params.get_mutable_abstract_parameter(index=0), model_abstract)
        # WARNING: This will invalidate old references!
        params.set_abstract_parameters(
            params.get_abstract_parameters().Clone())
        # WARNING: This may invalidate old references!
        params.SetFrom(copy.deepcopy(params))

        # Test alternative constructors.
        ctor_test = [
            Parameters(),
            Parameters(numeric=[model_numeric.Clone()]),
            Parameters(abstract=[model_abstract.Clone()]),
            Parameters(
                numeric=[model_numeric.Clone()],
                abstract=[model_abstract.Clone()]),
            Parameters(vec=model_numeric.Clone()),
            Parameters(value=model_abstract.Clone()),
            ]
