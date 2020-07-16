"""
Tests the pure Python functionality of `cpp_param`: (a) idempotent mapping for
unaliased types, and (b) correct mapping for aliased types (as the aliases
relate in C++).

N.B. The C++ types are not registered in this test. They are registered and
tested in `cpp_param_pybind_test`.
"""

import ctypes
import unittest

import numpy as np

import pydrake.common.cpp_param as mut


class CustomPyType:
    pass


class TestCppParam(unittest.TestCase):
    def _check_alias(self, canonical, alias):
        actual = mut.get_param_canonical([alias])[0]
        self.assertTrue(actual is canonical)

    def _check_idempotent(self, canonical):
        self._check_alias(canonical, canonical)

    def _check_aliases(self, canonical, aliases):
        for alias in aliases:
            self._check_alias(canonical, alias)

    def _check_names(self, name_canonical, aliases):
        for alias in aliases:
            actual = mut.get_param_names([alias])[0]
            self.assertEqual(actual, name_canonical)

    def test_idempotent(self):
        # Check idempotent mapping for unaliased types.
        # This follows the ordering in `cpp_param_pybind.cc`,
        # `RegisterCommon`.
        self._check_idempotent(bool)
        self._check_idempotent(str)
        self._check_idempotent(float)
        self._check_idempotent(np.float32)
        self._check_idempotent(int)
        self._check_idempotent(np.uint32)
        self._check_idempotent(np.int64)
        self._check_idempotent(object)
        # - Custom Types.
        self._check_idempotent(CustomPyType)
        # - Literals.
        self._check_idempotent(1)

    def test_aliases(self):
        # Aliases:
        # This follows the ordering in `cpp_param.py`,
        # `_ParamAliases._register_common`.
        self._check_aliases(float, [np.double, ctypes.c_double])
        self._check_aliases(np.float32, [ctypes.c_float])
        self._check_aliases(int, [np.int32, ctypes.c_int32])
        self._check_aliases(np.uint32, [ctypes.c_uint32])
        self._check_aliases(np.int64, [ctypes.c_int64])

    def test_names(self):
        self._check_names("int", [int, np.int32, ctypes.c_int32])
        self._check_names("CustomPyType", [CustomPyType])
        self._check_names("1", [1])
        self._check_names("List[CustomPyType]", [mut.List[CustomPyType]])
        self._check_names(
            "List[List[CustomPyType]]", [mut.List[mut.List[CustomPyType]]])
        # TODO(eric.cousineau): Hoist this if Dict[K, V] is ever needed for
        # registration.
        Dict = mut._Generic("Dict", dict, num_param=2)
        self._check_names("Dict[str, CustomPyType]", [Dict[str, CustomPyType]])

    def assert_equal_but_not_aliased(self, a, b):
        self.assertEqual(a, b)
        self.assertIsNot(a, b)

    def test_generic(self):
        empty = []
        self.assert_equal_but_not_aliased(mut.List[int](), empty)
        self.assert_equal_but_not_aliased(mut.List[int](empty), empty)
        nonempty = [1]
        self.assert_equal_but_not_aliased(mut.List[int](nonempty), nonempty)
        # N.B. This does not do any type checking at construction.
        nonempty_random = ["hello"]
        self.assert_equal_but_not_aliased(
            mut.List[int](nonempty_random), nonempty_random)
        # Ensure error message is good.
        with self.assertRaises(RuntimeError) as cm:
            mut.List[int, float]
        self.assertEqual(
            str(cm.exception),
            "<Generic List> can only accept 1 parameter(s)")
