import copy
import gc
import unittest

from pydrake.common.test.wrap_test_util import (
    MyContainerRawPtr,
    MyContainerUniquePtr,
    MyValue,
)


class TestWrapPybind(unittest.TestCase):
    def test_def_read_write_keep_alive(self):
        """Tests DefReadWriteKeepAlive."""
        c = MyContainerRawPtr()
        # Original value; we don't really care about its life.
        c.member = MyValue(value=42.)
        # Ensure that setter keeps the new pointee alive.
        member_b = MyValue(value=7.)
        c.member = member_b
        del member_b
        gc.collect()
        # Ensure that we have not lost our value.
        self.assertTrue(c.member.value == 7.)

    def test_def_read_unique_ptr(self):
        """Tests DefReadUniquePtr."""
        u = MyContainerUniquePtr(member=MyValue(value=42))
        # Ensure this is truly read-only.
        with self.assertRaises(AttributeError):
            u.member = None
        self.assertEqual(u.member.value, 42)
        # Get a reference-only view on the member.
        member = u.member
        # Ensure that the getter keeps the container alive.
        del u
        gc.collect()
        # Ensure that we have not lost our value.
        self.assertEqual(member.value, 42)
