from __future__ import print_function

import unittest
import ctypes
import numpy as np


from pydrake.util.cpp_types import get_types_canonical, get_type_names


class CustomPyType(object):
    pass


class TestCppTypes(unittest.TestCase):
    def _check_alias(self, alias, canonical):
        actual = get_types_canonical([alias])[0]
        self.assertTrue(actual is canonical)

    def _check_name(self, alias, name):
        actual = get_type_names([alias])[0]
        self.assertEquals(actual, name)

    def test_idempotent(self):
        # One-to-one.
        self._check_alias(bool, bool)
        self._check_alias(str, str)
        self._check_alias(int, int)
        self._check_alias(float, float)
        self._check_alias(object, object)
        self._check_name(int, "int")
        # - Custom Types.
        self._check_alias(CustomPyType, CustomPyType)
        self._check_name(CustomPyType, __name__ + ".CustomPyType")
        # - Literals.
        self._check_alias(1, 1)
        self._check_name(1, "1")

    def test_aliases(self):
        # Aliases:
        self._check_alias(np.double, float)
        self._check_alias(ctypes.c_int32, int)
        self._check_name(ctypes.c_int32, "int")


if __name__ == '__main__':
    unittest.main()
