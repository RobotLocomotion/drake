import copy
import gc
import unittest

from pydrake.common.test.wrap_test_util import (
    CheckTypeConversionExample,
    FunctionNeedsWrapCallbacks_Bad,
    FunctionNeedsWrapCallbacks,
    MakeTypeConversionExample,
    MakeTypeConversionExampleBadRvp,
    MyContainerRawPtr,
    MyContainerUniquePtr,
    MyValue,
    NotCopyable,
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
        u = MyContainerUniquePtr(member=MyValue(value=42),
                                 copyable_member=MyValue(value=9.7))
        # Ensure this is truly read-only.
        with self.assertRaises(AttributeError):
            u.member = None
        with self.assertRaises(AttributeError):
            u.copyable_member = None
        self.assertEqual(u.member.value, 42)
        self.assertEqual(u.copyable_member.value, 9.7)
        # Get a reference-only view on the member.
        member = u.member
        copyable_member = u.copyable_member
        # Ensure that the getter keeps the container alive.
        del u
        gc.collect()
        # Ensure that we have not lost our value.
        self.assertEqual(member.value, 42)
        self.assertEqual(copyable_member.value, 9.7)

    def test_type_caster_wrapped(self):
        value = MakeTypeConversionExample()
        self.assertIsInstance(value, str)
        self.assertEqual(value, "hello")
        with self.assertRaises(RuntimeError) as cm:
            MakeTypeConversionExampleBadRvp()
        self.assertEqual(
            str(cm.exception),
            "Can only pass TypeConversionExample by value.")
        self.assertTrue(CheckTypeConversionExample(obj=value))

    def test_wrap_callbacks(self):
        call_count = 0

        def callback(value):
            nonlocal call_count
            call_count += 1
            self.assertIsInstance(value, NotCopyable)

        # If we call our test function via a binding that does not
        # use WrapCallbacks(), we get an error about non-copyable types.
        with self.assertRaises(RuntimeError) as cm:
            FunctionNeedsWrapCallbacks_Bad(callback)
        self.assertIn("non-copyable", str(cm.exception))

        # Using the function with WrapCallbacks() should function as intended.
        loopback = FunctionNeedsWrapCallbacks(callback)
        self.assertEqual(call_count, 1)

        # When the function is looped back, it is not the same object (due to
        # extra std::function<> shims), but still ultimately calls `callback`.
        self.assertIsNot(loopback, callback)
        loopback(NotCopyable())
        self.assertEqual(call_count, 2)

        # Ensure that effectively passing an empty std::function<> does not
        # cause a bad_function exception.
        should_be_none = FunctionNeedsWrapCallbacks(None)
        self.assertIs(should_be_none, None)
