import copy
import unittest

from pydrake.common import pretty_class_name
from pydrake.common.cpp_param import List
from pydrake.common.value import AbstractValue, Value

from pydrake.common.test.value_test_util import (
    make_abstract_value_cc_type_unregistered,
    CustomType,
    MoveOnlyType,
)


class TestValue(unittest.TestCase):
    def test_module(self):
        # Note that __module__ is not what you expect.
        self.assertEqual(Value.__module__, "pydrake.common.cpp_template")
        # Instead, `get_module_name()` is what you want for instances of type
        # TemplateBase.
        self.assertEqual(Value.get_module_name(), "pydrake.common.value")

    def test_abstract_value_type_conversion(self):
        """Tests type-conversion types (passed by value, not reference)."""
        expected = "Hello world"
        value = Value[str](expected)
        self.assertIsInstance(value, AbstractValue)
        self.assertEqual(value.get_value(), expected)
        expected_new = "New value"
        value.set_value(expected_new)
        self.assertEqual(value.get_value(), expected_new)
        # Test docstring.
        self.assertFalse("unique_ptr" in value.set_value.__doc__)
        with self.assertRaises(RuntimeError) as cm:
            value.get_mutable_value()
        self.assertEqual(
            str(cm.exception),
            "Cannot get mutable value (or reference) for a type-conversion "
            "type: <class 'str'>")

    def test_abstract_value_registered_class(self):
        """Tests registered class types (passable by reference and value). Also
        tests a move-only class type."""
        obj = MoveOnlyType(10)
        self.assertEqual(
            pretty_class_name(Value[MoveOnlyType]),
            "Value[MoveOnlyType]")
        self.assertRegex(
            str(Value[MoveOnlyType]),
            "<class 'pydrake.*Value.*MoveOnlyType")
        # This *always* clones `obj`.
        value = Value[MoveOnlyType](obj)
        self.assertTrue(value.get_value() is not obj)
        self.assertEqual(value.get_value().x(), 10)
        # Set value (via `get_mutable_value`).
        value.get_mutable_value().set_x(20)
        self.assertEqual(value.get_value().x(), 20)
        # Test custom emplace constructor.
        emplace_value = Value[MoveOnlyType](30)
        self.assertEqual(emplace_value.get_value().x(), 30)
        # Test docstring.
        self.assertTrue("unique_ptr" in value.set_value.__doc__)

    def assert_equal_but_not_aliased(self, a, b):
        self.assertEqual(a, b)
        self.assertIsNot(a, b)

    def test_assert_equal_but_not_aliased(self):
        """
        This is a higher-level test for `assert_equal_but_not_aliased`, not for
        the device under test `Value[]`.
        """
        a = {"x": 10}
        b = copy.deepcopy(a)
        self.assert_equal_but_not_aliased(a, b)
        # Confirm that mutation will not affect the other copy.
        b["x"] = 100
        self.assertNotEqual(a, b)

        # Negatives.
        # - Self-identity should fail.
        with self.assertRaises(AssertionError):
            self.assert_equal_but_not_aliased(a, a)
        # - Inequality should fail.
        with self.assertRaises(AssertionError):
            self.assert_equal_but_not_aliased(a, {"x": -10})

    def test_abstract_value_py_object(self):
        # The Value constructor operates by taking a reference, not by copying.
        initial_dict = {"x": 10}
        value = Value[object](initial_dict)
        self.assertIs(value.get_value(), initial_dict)
        self.assertIs(value.get_mutable_value(), initial_dict)

        # Therefore, mutating via the accessor will also mutate the original.
        value.get_mutable_value()["y"] = 20
        self.assertEqual(value.get_value(), {"x": 10, "y": 20})
        self.assertEqual(initial_dict, {"x": 10, "y": 20})

        # Cloning the value should perform a deep copy of the Python object.
        cloned_value = copy.deepcopy(value)
        cloned_dict = cloned_value.get_value()
        self.assert_equal_but_not_aliased(cloned_dict, initial_dict)

        # For both the original Value[] and the cloned Value[], mutating via
        # the accessor should not influence the other one.
        value.get_mutable_value()["y"] = 55
        cloned_value.get_mutable_value()["y"] = 99
        self.assertEqual(value.get_value(), {"x": 10, "y": 55})
        self.assertEqual(initial_dict, {"x": 10, "y": 55})
        self.assertEqual(cloned_value.get_value(), {"x": 10, "y": 99})
        self.assertEqual(cloned_dict, {"x": 10, "y": 99})

        # Using `set_value` on the original Value[] should update the object
        # reference to the new value. The clone is unchanged.
        new_dict = {"a": 444}
        value.set_value(new_dict)
        self.assertEqual(new_dict, {"a": 444})
        self.assertIs(value.get_value(), new_dict)
        self.assertIs(value.get_mutable_value(), new_dict)
        self.assertEqual(cloned_value.get_value(), {"x": 10, "y": 99})

        # Using `SetFrom` should perform a deep copy, not just alias (#18653).
        value.SetFrom(cloned_value)
        self.assertIs(cloned_value.get_value(), cloned_dict)
        self.assert_equal_but_not_aliased(value.get_value(), cloned_dict)

    def test_abstract_value_py_list(self):
        value_cls = Value[List[CustomType]]
        empty = []
        nonempty = [CustomType()]
        # Empty construction.
        value = value_cls()
        self.assertEqual(value.get_value(), empty)
        value = value_cls(empty)
        self.assert_equal_but_not_aliased(value.get_value(), empty)
        value = value_cls(nonempty)
        self.assert_equal_but_not_aliased(value.get_value(), nonempty)
        # Test mutation.
        value = value_cls(nonempty)
        self.assertNotEqual(value.get_value(), empty)
        value.set_value(empty)
        self.assert_equal_but_not_aliased(value.get_value(), empty)

    def test_abstract_value_make(self):
        value = AbstractValue.Make("Hello world")
        self.assertIsInstance(value, Value[str])
        value = AbstractValue.Make(1.0)
        self.assertIsInstance(value, Value[float])
        value = AbstractValue.Make(MoveOnlyType(10))
        self.assertIsInstance(value, Value[MoveOnlyType])
        value = AbstractValue.Make({"x": 10})
        self.assertIsInstance(value, Value[object])
        # N.B. Empty lists cannot have their type inferred, so the type will
        # default to `Value[object]`.
        value = AbstractValue.Make([])
        self.assertIsInstance(value, Value[object])
        # N.B. Non-empty lists can have their type inferred.
        value = AbstractValue.Make([CustomType()])
        self.assertIsInstance(value, Value[List[CustomType]])

    def test_abstract_value_cc_type_unregistered(self):
        value = make_abstract_value_cc_type_unregistered()
        self.assertIsInstance(value, AbstractValue)
        with self.assertRaises(RuntimeError) as cm:
            value.get_value()
        self.assertTrue(all(
            s in str(cm.exception) for s in [
                "AbstractValue",
                "UnregisteredType",
                "get_value",
                "AddValueInstantiation",
            ]), cm.exception)

    def test_value_registration(self):
        # Existence check.
        Value[object]
        Value[str]
        Value[bool]

    def test_value_ctor(self):
        self.assertIsInstance(Value("foo"), Value[str])
        self.assertIsInstance(Value(True), Value[bool])
        self.assertIsInstance(Value(1.0), Value[float])
        self.assertIsInstance(Value(object()), Value[object])
