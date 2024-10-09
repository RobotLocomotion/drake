"""Unit test for ref_cycle<>() annotation.

See also ref_cycle_test_util_py.cc for the bindings used in the tests.
"""
import functools
import gc
import sys
import unittest
import weakref

from pydrake.common.ref_cycle_test_util import (
    NotDynamic, IsDynamic, invalid_arg_index, free_function, ouroboros)


def actual_ref_count(o):
    """Returns the actual ref count of `o`, in the caller's scope."""
    # sys.getrefcount() always artificially adds 1 to its result, owing to the
    # machinery of python calling the native-code implementation. Since we wrap
    # it here, we have to adjust the result to account for the
    # python-implemented function call. For extra fun, the ref-count cost of a
    # python function call varies with python interpreter versions, so
    # wrapped_refcount_cost() actually measures the total (native call plus
    # python call) cost.
    @functools.cache
    def wrapped_refcount_cost():
        def wrapped(o):
            return sys.getrefcount(o)
        return wrapped(object())

    return sys.getrefcount(o) - wrapped_refcount_cost()


class TestRefCycle(unittest.TestCase):
    def check_is_collectable_cycle(self, p0, p1):
        # The edges of the cycle are:
        # p0 -> p0.__dict__ -> p0._pydrake_internal_ref_cycle_peers \
        #  -> p1 -> p1.__dict__ -> p1._pydrake_internal_ref_cycle_peers -> p0
        # where the object at each _pydrake_internal_ref_cycle_peers is a set.
        #
        # It is impractical to check the counts of p0 and p1 here because
        # callers may hold an arbitrary number of references.

        for x in [p0, p1]:
            self.assertEqual(actual_ref_count(x.__dict__), 1)
            self.assertEqual(
                actual_ref_count(x._pydrake_internal_ref_cycle_peers), 1)

            # Check that all parts are tracked by gc.
            self.assertTrue(gc.is_tracked(x))
            self.assertTrue(gc.is_tracked(x.__dict__))
            self.assertTrue(gc.is_tracked(x._pydrake_internal_ref_cycle_peers))

        # Check that the peers refer to each other.
        self.assertTrue(p1 in p0._pydrake_internal_ref_cycle_peers)
        self.assertTrue(p0 in p1._pydrake_internal_ref_cycle_peers)

    def check_no_cycle(self, p0, p1):
        for x in [p0, p1]:
            self.assertFalse(hasattr(x, '_pydrake_internal_ref_cycle_peers'))

    def test_invalid_index(self):
        with self.assertRaisesRegex(RuntimeError,
                                    "Could not activate ref_cycle.*"):
            invalid_arg_index()

    def test_ouroboros(self):
        # The self-cycle edges are:
        # dut -> dut.__dict__ -> dut._pydrake_internal_ref_cycle_peers -> dut
        #
        # This still passes check_is_collectable_cycle() -- the function just
        # does redundant work.
        dut = IsDynamic()
        returned = ouroboros(dut)
        self.assertEqual(returned, dut)
        self.assertEqual(len(dut._pydrake_internal_ref_cycle_peers), 1)
        self.check_is_collectable_cycle(returned, dut)

    def test_free_function(self):
        p0 = IsDynamic()
        p1 = IsDynamic()
        free_function(p0, p1)
        self.check_is_collectable_cycle(p0, p1)

    def test_not_dynamic_add(self):
        dut = NotDynamic()
        peer = IsDynamic()
        # Un-annotated call is fine.
        dut.AddIs(peer)
        self.check_no_cycle(dut, peer)
        # Annotated call dies because dut is not py::dynamic_attr().
        with self.assertRaisesRegex(
                RuntimeError, ".type.*index 1.*AddIsCycle.*not tracked.*"):
            dut.AddIsCycle(peer)

    def test_not_dynamic_return(self):
        dut = NotDynamic()
        # Un-annotated call is fine.
        returned = dut.ReturnIs()
        self.check_no_cycle(dut, returned)
        # Annotated call dies because dut is not py::dynamic_attr().
        with self.assertRaisesRegex(
                RuntimeError, ".type.*index 1.*ReturnIsCycle.*not tracked.*"):
            dut.ReturnIsCycle()

    def test_not_dynamic_null(self):
        dut = NotDynamic()
        # Un-annotated call is fine.
        self.assertIsNone(dut.ReturnNullIs())
        # Annotated call does not die because one peer is missing.
        self.assertIsNone(dut.ReturnNullIsCycle())

    def test_is_dynamic_add_not(self):
        dut = IsDynamic()
        notpeer = NotDynamic()
        dut.AddNot(notpeer)
        self.check_no_cycle(dut, notpeer)
        # Annotated call dies because notpeer is not py::dynamic_attr().
        with self.assertRaisesRegex(
                RuntimeError, ".type.*index 2.*AddNotCycle.*not tracked.*"):
            dut.AddNotCycle(notpeer)

    def test_is_dynamic_return_not(self):
        dut = IsDynamic()
        # Un-annotated call is fine.
        returned = dut.ReturnNot()
        self.check_no_cycle(dut, returned)
        # Annotated call dies because return is not py::dynamic_attr().
        with self.assertRaisesRegex(
                RuntimeError, ".type.*index 0.*ReturnNotCycle.*not tracked.*"):
            dut.ReturnNotCycle()

    def test_is_dynamic_return_null(self):
        dut = IsDynamic()
        # Un-annotated call is fine.
        self.assertIsNone(dut.ReturnNullNot())
        self.assertIsNone(dut.ReturnNullIs())
        # Annotated call does not die because one peer is missing.
        self.assertIsNone(dut.ReturnNullNotCycle())
        self.assertIsNone(dut.ReturnNullIsCycle())

    def test_is_dynamic_add_is(self):
        dut = IsDynamic()
        peer = IsDynamic()
        # Un-annotated call does not implement a cycle.
        dut.AddIs(peer)
        self.check_no_cycle(dut, peer)
        # Annotated call produces a collectable cycle.
        dut.AddIsCycle(peer)
        self.check_is_collectable_cycle(dut, peer)

    def test_is_dynamic_return_is(self):
        dut = IsDynamic()
        # Un-annotated call does not implement a cycle.
        returned = dut.ReturnIs()
        self.check_no_cycle(dut, returned)
        # Annotated call produces a collectable cycle.
        returned = dut.ReturnIsCycle()
        self.check_is_collectable_cycle(dut, returned)

    def test_actual_collection(self):

        def make_a_cycle():
            dut = IsDynamic()
            return dut.ReturnIsCycle()

        cycle = make_a_cycle()
        finalizer = weakref.finalize(cycle, lambda: None)
        # Cycle is alive while we refer to it.
        self.assertTrue(finalizer.alive)
        del cycle
        # Cycle is alive because of the ref_cycle.
        self.assertTrue(finalizer.alive)
        gc.collect()
        # Cycle does not survive garbage collection.
        self.assertFalse(finalizer.alive)
