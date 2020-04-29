import copy
import unittest

import numpy as np

from pydrake.common.value import (
    AbstractValue, Value, VectorX, MatrixX)

from pydrake.common.test.value_test_util import (
    make_abstract_value_cc_type_unregistered,
    MoveOnlyType,
)
from pydrake.common.test_utilities import numpy_compare


class TestValue(unittest.TestCase):
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

    def test_eigen_placeholder_api(self):
        VectorXd = VectorX[float]
        # TODO: Fix module name? Should not be `cpp_template`.
        self.assertEqual(
            str(VectorXd),
            f"<class 'pydrake.common.cpp_template.VectorX[float]'>")
        self.assertEqual(
            str(Value[VectorXd]),
            f"<class 'pydrake.common.value.Value[VectorX[float]]'>")
        with self.assertRaises(NotImplementedError):
            VectorXd()

        MatrixXd = MatrixX[float]
        self.assertEqual(
            str(MatrixXd),
            f"<class 'pydrake.common.cpp_template.MatrixX[float]'>")
        self.assertEqual(
            str(Value[MatrixXd]),
            f"<class 'pydrake.common.value.Value[MatrixX[float]]'>")
        with self.assertRaises(NotImplementedError):
            MatrixXd()

    @numpy_compare.check_all_types
    def test_eigen_value_existence(self, T):
        VectorX[T]
        Value[VectorX[T]]

    def test_eigen_value_api(self):
        VectorXd = VectorX[float]
        value = Value[VectorXd]()
        x = np.array([1., 2, 3])
        value.set_value(x)
        numpy_compare.assert_equal(value.get_value(), x)
        numpy_compare.assert_equal(value.get_mutable_value(), x)
        self.assertIsInstance(AbstractValue.Make(x), Value[VectorXd])

        MatrixXd = MatrixX[float]
        value = Value[MatrixXd]()
        x = np.eye(3)
        value.set_value(x)
        numpy_compare.assert_equal(value.get_value(), x)
        numpy_compare.assert_equal(value.get_mutable_value(), x)
        self.assertIsInstance(AbstractValue.Make(x), Value[MatrixXd])
