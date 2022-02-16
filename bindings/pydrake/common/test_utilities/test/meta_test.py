import pydrake.common.test_utilities.meta as mut

import unittest


class TestMeta(unittest.TestCase):

    def _test_chosen_suffixes(self, expected_pairs):
        """Given an expected list of (suffix, kwargs) pairs to be returned by
        _make_test_pairs, checks that when the kwargs are actually passed into
        _make_test_pairs that it returns those desired pairs.
        """
        dut = mut._make_test_pairs
        with self.subTest(pairs=expected_pairs):
            values = [x for _, x in expected_pairs]
            actual_pairs = dut(values)
            self.assertSequenceEqual(actual_pairs, expected_pairs)

    def test_naming_basic(self):
        """Checks the basic case from the meta.py docstring."""
        self._test_chosen_suffixes((
            ("22", dict(foo=22)),
            ("33", dict(foo=33)),
        ))

    def test_naming_two_args(self):
        """Checks where each kwargs has multiple entries."""
        self._test_chosen_suffixes((
            ("22_hello", dict(foo=22, bar="hello")),
            ("33_hello", dict(foo=33, bar="hello")),
        ))

    def test_conflicting_names(self):
        """Checks where the suffix would not be unique without a nonce, and
        that multiple unprintable characters in a row are combined into just
        one underscore.
        """
        self._test_chosen_suffixes((
            ("foo_bar_iter0", dict(quux="foo//bar")),
            ("foo_bar_iter1", dict(quux="foo::bar")),
        ))

    def test_short_names(self):
        """Checks where the value is totally non-printable."""
        # The leading "_" here is unfortunate, but we can live with it.
        self._test_chosen_suffixes((
            ("_iter0", dict(quux="...")),
        ))
