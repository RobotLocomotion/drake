# -*- coding: utf-8 -*-

import copy
import unittest

import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.common.value import AbstractValue, Value
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

                value_data = BasicVector(expected_init)
                value = value_data.get_mutable_value()
                self.assertTrue(np.allclose(value, expected_init))

                # Add value directly.
                # TODO(eric.cousineau): Determine if there is a way to extract
                # the pointer referred to by the buffer (e.g. `value.data`).
                value[:] += 1
                self.assertTrue(np.allclose(value, expected_add))
                self.assertTrue(
                    np.allclose(value_data.get_value(), expected_add))
                self.assertTrue(
                    np.allclose(value_data.get_mutable_value(), expected_add))

                # Set value from `BasicVector`.
                value_data.SetFromVector(expected_set)
                self.assertTrue(np.allclose(value, expected_set))
                self.assertTrue(
                    np.allclose(value_data.get_value(), expected_set))
                self.assertTrue(
                    np.allclose(value_data.get_mutable_value(), expected_set))
                # Ensure we can construct from size.
                value_data = BasicVector(n)
                self.assertEqual(value_data.size(), n)
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
        self.assertEqual(value.GetAtIndex(1), 4.)
        value.SetAtIndex(1, 5.)
        self.assertEqual(value.GetAtIndex(1), 5.)

    def test_value_registration(self):
        for T in [float, AutoDiffXd, Expression]:
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
