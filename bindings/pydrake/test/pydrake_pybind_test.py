"""Test binding helper methods from `pydrake_pybind_test`.
See `pydrake_pybind_test_util_py.cc` the bindings used in these tests.
"""

import copy
import unittest
import weakref

from pydrake.pydrake_pybind_test_util import (
    ExampleDefClone,
    ExampleDefCopyAndDeepCopy,
    ExampleParamInit,
    ExamplePyKeepAlive,
    Item,
)


def check_copy(copy_function, obj):
    """Checks `copy_function` to ensure `obj` is equal to its copy, and that
    it is not the same instance."""
    obj_copy = copy_function(obj)
    return obj == obj_copy and obj is not obj_copy


def check_reference_internal_list(cls):
    """
    Expicitly check behavior of `py_rvp::reference_internal` on a return list
    of pointers given class definitions used in `pydrake_pybind_test.cc`.
    """
    container = cls()
    ref = weakref.ref(container)
    item = container.a_list()[0]
    del container
    # Show that `item` is effectively keeping `container` alive.
    assert ref() is not None, "Container has been gc'd!"
    # Show that removing refcount from `item` will cause `container` to get
    # garbage collected.
    del item
    assert ref() is None, "Container has *not* been gc'd!"
    return True


class TestPydrakePybindFundamentals(unittest.TestCase):
    def test_py_keep_alive(self):
        self.assertTrue(isinstance(ExamplePyKeepAlive().a(), Item))
        self.assertEqual(ExamplePyKeepAlive().a().value, 10)
        self.assertIsInstance(ExamplePyKeepAlive().a_list(), list)
        self.assertEqual(len(ExamplePyKeepAlive().a_list()), 1)
        self.assertIsInstance(ExamplePyKeepAlive().a_list()[0], Item)

        # Ensure we test the value to check for memory corruption.
        self.assertEqual(ExamplePyKeepAlive().a_list()[0].value, 10)
        # Explicitly test keep alive behavior.
        self.assertTrue(check_reference_internal_list(cls=ExamplePyKeepAlive))

    def test_py_def_copy_and_deepcopy(self):
        self.assertTrue(check_copy(copy.copy, ExampleDefCopyAndDeepCopy(10)))
        self.assertTrue(
            check_copy(copy.deepcopy, ExampleDefCopyAndDeepCopy(20))
        )

    def test_py_def_clone(self):
        self.assertTrue(check_copy(ExampleDefClone.Clone, ExampleDefClone(5)))
        self.assertTrue(check_copy(copy.copy, ExampleDefClone(10)))
        self.assertTrue(check_copy(copy.deepcopy, ExampleDefClone(20)))

    def test_py_param_init(self):
        self.assertTrue(ExampleParamInit().compare_values(0, 1))
        self.assertTrue(ExampleParamInit(a=10).compare_values(10, 1))
        self.assertTrue(ExampleParamInit(b=20).compare_values(0, 20))
        self.assertTrue(ExampleParamInit(a=10, b=20).compare_values(10, 20))
