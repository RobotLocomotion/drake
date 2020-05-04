import copy
import unittest

from pydrake.common.value import AbstractValue, Value

from pydrake.common.test.value_test_util import (
    make_unknown_abstract_value,
    MoveOnlyType,
)


class TestValue(unittest.TestCase):
    def test_abstract_value_copyable(self):
        """Tests copying, and type-conversion types."""
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

    def test_abstract_value_move_only(self):
        """Tests non-type-conversion types."""
        obj = MoveOnlyType(10)
        self.assertEqual(
            str(Value[MoveOnlyType]),
            "<class 'pydrake.common.value.Value[MoveOnlyType]'>")
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

    def test_abstract_value_py_object(self):
        expected = {"x": 10}
        value = Value[object](expected)
        # Value is by reference, *not* by copy.
        self.assertTrue(value.get_value() is expected)
        # Update mutable version.
        value.get_mutable_value()["y"] = 30
        self.assertEqual(value.get_value(), expected)
        # Cloning the value should perform a deep copy of the Python object.
        value_clone = copy.deepcopy(value)
        self.assertEqual(value_clone.get_value(), expected)
        self.assertTrue(value_clone.get_value() is not expected)
        # Using `set_value` on the original value changes object reference.
        expected_new = {"a": 20}
        value.set_value(expected_new)
        self.assertEqual(value.get_value(), expected_new)
        self.assertTrue(value.get_value() is not expected)

    def test_abstract_value_make(self):
        value = AbstractValue.Make("Hello world")
        self.assertIsInstance(value, Value[str])
        value = AbstractValue.Make(MoveOnlyType(10))
        self.assertIsInstance(value, Value[MoveOnlyType])
        value = AbstractValue.Make({"x": 10})
        self.assertIsInstance(value, Value[object])

    def test_abstract_value_unknown(self):
        value = make_unknown_abstract_value()
        self.assertIsInstance(value, AbstractValue)
        with self.assertRaises(RuntimeError) as cm:
            value.get_value()
        self.assertTrue(all(
            s in str(cm.exception) for s in [
                "AbstractValue",
                "UnknownType",
                "get_value",
                "AddValueInstantiation",
            ]), cm.exception)

    def test_value_registration(self):
        # Existence check.
        Value[object]
        Value[str]
        Value[bool]
