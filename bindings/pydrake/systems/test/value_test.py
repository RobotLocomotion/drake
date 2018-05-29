# -*- coding: utf-8 -*-

from __future__ import print_function

import copy
import unittest
import numpy as np

from pydrake.systems.framework import (
    AbstractValue,
    BasicVector,
    Parameters,
    Value,
    VectorBase,
    )
from pydrake.systems.test.test_util import (
    make_unknown_abstract_value,
    MoveOnlyType,
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
                expected_init = wrap(map(float, range(n)))
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
                self.assertEquals(value_data.size(), n)
                # Ensure we can clone.
                value_copies = [
                    value_data.Clone(),
                    copy.copy(value_data),
                    copy.deepcopy(value_data),
                ]
                for value_copy in value_copies:
                    self.assertTrue(value_copy is not value_data)
                    self.assertEquals(value_data.size(), n)

    def test_basic_vector_set_get(self):
        value = BasicVector(np.arange(3., 5.))
        self.assertEquals(value.GetAtIndex(1), 4.)
        value.SetAtIndex(1, 5.)
        self.assertEquals(value.GetAtIndex(1), 5.)

    def test_abstract_value_copyable(self):
        expected = "Hello world"
        value = Value[str](expected)
        self.assertTrue(isinstance(value, AbstractValue))
        self.assertEquals(value.get_value(), expected)
        expected_new = "New value"
        value.set_value(expected_new)
        self.assertEquals(value.get_value(), expected_new)
        # Test docstring.
        self.assertFalse("unique_ptr" in value.set_value.__doc__)

    def test_abstract_value_move_only(self):
        obj = MoveOnlyType(10)
        # This *always* clones `obj`.
        self.assertEquals(
            str(Value[MoveOnlyType]),
            "<class 'pydrake.systems.framework.Value[MoveOnlyType]'>")
        value = Value[MoveOnlyType](obj)
        self.assertTrue(value.get_value() is not obj)
        self.assertEquals(value.get_value().x(), 10)
        # Set value.
        value.get_mutable_value().set_x(20)
        self.assertEquals(value.get_value().x(), 20)
        # Test custom emplace constructor.
        emplace_value = Value[MoveOnlyType](30)
        self.assertEquals(emplace_value.get_value().x(), 30)
        # Test docstring.
        self.assertTrue("unique_ptr" in value.set_value.__doc__)

    def test_abstract_value_py_object(self):
        expected = {"x": 10}
        value = Value[object](expected)
        # Value is by reference, *not* by copy.
        self.assertTrue(value.get_value() is expected)
        # Update mutable version.
        value.get_mutable_value()["y"] = 30
        self.assertEquals(value.get_value(), expected)
        # Cloning the value should perform a deep copy of the Python object.
        value_clone = copy.deepcopy(value)
        self.assertEquals(value_clone.get_value(), expected)
        self.assertTrue(value_clone.get_value() is not expected)
        # Using `set_value` on the original value changes object reference.
        expected_new = {"a": 20}
        value.set_value(expected_new)
        self.assertEquals(value.get_value(), expected_new)
        self.assertTrue(value.get_value() is not expected)

    def test_abstract_value_make(self):
        value = AbstractValue.Make("Hello world")
        self.assertTrue(isinstance(value, Value[str]))
        value = AbstractValue.Make(MoveOnlyType(10))
        self.assertTrue(isinstance(value, Value[MoveOnlyType]))
        value = AbstractValue.Make({"x": 10})
        self.assertTrue(isinstance(value, Value[object]))

    def test_abstract_value_unknown(self):
        value = make_unknown_abstract_value()
        self.assertTrue(isinstance(value, AbstractValue))
        with self.assertRaises(RuntimeError) as cm:
            value.get_value()
        self.assertTrue(all(
            s in cm.exception.message for s in [
                "AbstractValue",
                "UnknownType",
                "get_value",
                "AddValueInstantiation",
            ]), cm.exception.message)

    def test_parameters_api(self):

        def compare(actual, expected):
            self.assertEquals(type(actual), type(expected))
            if isinstance(actual, VectorBase):
                self.assertTrue(
                    np.allclose(actual.get_value(), expected.get_value()))
            else:
                self.assertEquals(actual.get_value(), expected.get_value())

        model_numeric = BasicVector([0.])
        model_abstract = AbstractValue.Make("Hello")

        params = Parameters(
            numeric=[model_numeric.Clone()], abstract=[model_abstract.Clone()])
        self.assertEquals(params.num_numeric_parameters(), 1)
        self.assertEquals(params.num_abstract_parameters(), 1)
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
