"""Unit test for drake::SortedPair<T> type bindings.

See also C++ testing support in sorted_pair_test_util_py.cc.
"""

import unittest

from pydrake.common.sorted_pair_test_util import (
    PassThrough,
    PassThroughMap,
)


class TestSortedPair(unittest.TestCase):
    def test_unsorted_pair_input(self):
        # Check single value. Pass in an unsorted pair, and confirm that we get
        # a sorted pair out.
        self.assertEqual(PassThrough(("b", "a")), ("a", "b"))

    def test_composite_type(self):
        # Check compsite type.
        self.assertEqual(PassThroughMap({("b", "a"): 10}), {("a", "b"): 10})
