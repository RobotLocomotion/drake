"""Tests the behavior of TypeSafeIndex<T> type bindings.  See
`type_safe_index_test_util_py.cc` for test-only bindings and additional C++
tests.
"""

import unittest

from pydrake.common.type_safe_index_test_util import (
    Index,
    OtherIndex,
    pass_thru_index,
    pass_thru_int,
)
from pydrake.common.value import Value


class TestTypeSafeIndex(unittest.TestCase):
    def test_index_basics(self):
        self.assertTrue(Index(10).is_valid())
        self.assertFalse(Index().is_valid())
        self.assertEqual(pass_thru_int(10), 10)
        self.assertEqual(pass_thru_int(Index(10)), 10)

        # TypeSafeIndex<> is not implicitly constructible from an int.
        with self.assertRaises(TypeError):
            pass_thru_index(10)
        self.assertEqual(pass_thru_index(Index(10)), 10)
        self.assertEqual(pass_thru_index(Index(10)), Index(10))

    def test_index_types_are_distinct(self):
        with self.assertRaises(TypeError):
            pass_thru_index(OtherIndex(10))

    def test_comparison(self):
        self.assertEqual(Index(10), Index(10))
        self.assertEqual(Index(10), 10)
        self.assertEqual(10, Index(10))
        self.assertLess(Index(9), Index(10))
        # TODO(rpoyner-tri): presently only assertLess() works with this type,
        # and related inequality test methods do not. Should they?
        self.assertFalse(Index(11) < Index(10))

    def test_hash(self):
        # Store values for hash computation so that their id()s cannot be
        # recycled by the GC; otherwise, if the correct `__hash__`
        # implementation were missing and it used the default implementation
        # (using `id()`), we'd get a false positive.
        a = Index(10)
        b = Index(10)
        c = Index(9)
        self.assertEqual(hash(a), hash(b))
        self.assertNotEqual(hash(a), hash(c))

    def test_repr(self):
        self.assertEqual(repr(Index(10)), "Index(10)")

    def test_value_instantiation(self):
        self.assertIsInstance(Value(Index(11)).get_value(), Index)
