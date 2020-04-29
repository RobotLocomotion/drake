import copy
import gc
import unittest

from pydrake.common.wrap_test_util import (
    MyContainer,
    MyUniquePtr,
    MyValue,
)


class TestWrapPybind(unittest.TestCase):
    def test_read_write_keep_alive(self):
        # Original value; we don't really care about its life.
        value_a = MyValue()
        value_a.value = 42.
        c = MyContainer()
        c.member = value_a
        # Ensure that setter keeps the new pointee alive.
        value_b = MyValue()
        value_b.value = 7.
        c.member = value_b
        del value_b
        gc.collect()
        # Ensure that we have not lost our value.
        self.assertTrue(c.member.value == 7.)

    def test_unique_ptr_keep_alive(self):
        u = MyUniquePtr(42)
        self.assertEqual(u.member, 42)
        val = u.member
        # Ensure that the getter keeps the container alive.
        del u
        gc.collect()
        # Ensure that we have not lost our value.
        self.assertEqual(val, 42)
