"""Unit test for builder_life_support_stash<>() annotation.

See also systems/test/test_util_py.cc for the bindings used in the tests.
"""
import gc
import unittest
import weakref

from pydrake.common.test_utilities import numpy_compare
from pydrake.common.test_utilities.memory_test_util import actual_ref_count
from pydrake.systems.test.test_util import (
    Arbitrary, DiagramBuilderTestAdversary_)


class TestBuilderLifeSupport(unittest.TestCase):
    @numpy_compare.check_all_types
    def test_invalid_index(self, T):
        DiagramBuilderTestAdversary = DiagramBuilderTestAdversary_[T]
        adversary = DiagramBuilderTestAdversary()
        with self.assertRaisesRegex(
                RuntimeError,
                "Could not activate builder_life_support_stash.*"):
            adversary.StashBadIndex()
        self.assertEqual(actual_ref_count(adversary), 1)

    @numpy_compare.check_all_types
    def test_wrong_type(self, T):
        DiagramBuilderTestAdversary = DiagramBuilderTestAdversary_[T]
        adversary = DiagramBuilderTestAdversary()
        with self.assertRaisesRegex(RuntimeError, "Unable to cast.*"):
            adversary.StashWrongType(Arbitrary())
        self.assertEqual(actual_ref_count(adversary), 1)

    @numpy_compare.check_all_types
    def test_bind_at_init(self, T):
        DiagramBuilderTestAdversary = DiagramBuilderTestAdversary_[T]
        # The int-accepting constructor also stashes.
        adversary = DiagramBuilderTestAdversary(10)
        self.assertEqual(actual_ref_count(adversary), 2)
        adversary.Abandon()
        self.assertEqual(actual_ref_count(adversary), 1)

    @numpy_compare.check_all_types
    def test_bind_self(self, T):
        DiagramBuilderTestAdversary = DiagramBuilderTestAdversary_[T]
        adversary = DiagramBuilderTestAdversary()
        adversary.StashSelf()
        self.assertEqual(actual_ref_count(adversary), 2)
        adversary.Abandon()
        self.assertEqual(actual_ref_count(adversary), 1)

    @numpy_compare.check_all_types
    def test_bind_returned_self(self, T):
        DiagramBuilderTestAdversary = DiagramBuilderTestAdversary_[T]
        adversary = DiagramBuilderTestAdversary()
        adversary.StashReturnedSelf()
        self.assertEqual(actual_ref_count(adversary), 2)
        adversary.Abandon()
        self.assertEqual(actual_ref_count(adversary), 1)

    @numpy_compare.check_all_types
    def test_bind_returned_none(self, T):
        DiagramBuilderTestAdversary = DiagramBuilderTestAdversary_[T]
        adversary = DiagramBuilderTestAdversary()
        adversary.StashReturnedNull()
        self.assertEqual(actual_ref_count(adversary), 1)
