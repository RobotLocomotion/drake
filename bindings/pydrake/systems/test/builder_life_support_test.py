"""Unit test for builder_life_support_stash<>() annotation.

See also systems/test/test_util_py.cc for the bindings used in the tests.
"""

import sys
import unittest

from pydrake.common.test_utilities import numpy_compare
from pydrake.systems.test.test_util import (
    Arbitrary,
    DiagramBuilderTestAdversary_,
)


class TestBuilderLifeSupport(unittest.TestCase):
    @numpy_compare.check_all_types
    def test_invalid_index(self, T):
        DiagramBuilderTestAdversary = DiagramBuilderTestAdversary_[T]
        adversary = DiagramBuilderTestAdversary()
        with self.assertRaisesRegex(
            RuntimeError, "Could not activate builder_life_support_stash.*"
        ):
            adversary.StashBadIndex()

    @numpy_compare.check_all_types
    def test_wrong_type(self, T):
        DiagramBuilderTestAdversary = DiagramBuilderTestAdversary_[T]
        adversary = DiagramBuilderTestAdversary()
        with self.assertRaisesRegex(RuntimeError, "Unable to cast.*"):
            adversary.StashWrongType(Arbitrary())

    @numpy_compare.check_all_types
    def test_bind_at_init(self, T):
        DiagramBuilderTestAdversary = DiagramBuilderTestAdversary_[T]
        # The int-accepting constructor also stashes.
        adversary = DiagramBuilderTestAdversary(10)
        # Python versions differ on whether binding an object to a local
        # variable increases the ref count.
        tare = sys.getrefcount(adversary)
        adversary.Abandon()
        self.assertEqual(sys.getrefcount(adversary), tare - 1)

    @numpy_compare.check_all_types
    def test_bind_self(self, T):
        DiagramBuilderTestAdversary = DiagramBuilderTestAdversary_[T]
        adversary = DiagramBuilderTestAdversary()
        # Python versions differ on whether binding an object to a local
        # variable increases the ref count.
        tare = sys.getrefcount(adversary)
        adversary.StashSelf()
        self.assertEqual(sys.getrefcount(adversary), tare + 1)
        adversary.Abandon()
        self.assertEqual(sys.getrefcount(adversary), tare)

    @numpy_compare.check_all_types
    def test_bind_returned_self(self, T):
        DiagramBuilderTestAdversary = DiagramBuilderTestAdversary_[T]
        adversary = DiagramBuilderTestAdversary()
        # Python versions differ on whether binding an object to a local
        # variable increases the ref count.
        tare = sys.getrefcount(adversary)
        adversary.StashReturnedSelf()
        self.assertEqual(sys.getrefcount(adversary), tare + 1)
        adversary.Abandon()
        self.assertEqual(sys.getrefcount(adversary), tare)

    @numpy_compare.check_all_types
    def test_bind_returned_none(self, T):
        DiagramBuilderTestAdversary = DiagramBuilderTestAdversary_[T]
        adversary = DiagramBuilderTestAdversary()
        # Python versions differ on whether binding an object to a local
        # variable increases the ref count.
        tare = sys.getrefcount(adversary)
        adversary.StashReturnedNull()
        self.assertEqual(sys.getrefcount(adversary), tare)
