from pydrake.util.pure_hash_dict import PureHashDict

import unittest


class Comparison(object):
    def __init__(self, lhs, rhs):
        self.lhs = lhs
        self.rhs = rhs

    def __nonzero__(self):
        raise ValueError("Should not be called")


class Item(object):
    def __init__(self, value):
        self.value = value

    def __hash__(self):
        return hash(self.value)

    def __eq__(self, other):
        # Non-boolean return value.
        return Comparison(self.value, other.value)


# Globals for testing.
a = Item(1)
b = Item(2)


class TestPureHashDict(unittest.TestCase):
    def test_normal_dict(self):
        d = {a: "a", b: "b"}
        # TODO(eric.cousineau): Figure out how to reproduce failure when `dict`
        # attempts to use `__eq__`, similar to what happens when using
        # `Polynomial` as a key in a dictionary.
        self.assertEqual(d[a], "a")
        with self.assertRaises(ValueError):
            value = bool(a == b)

    def test_pure_hash_dict(self):
        d = PureHashDict({a: "a", b: "b"})
        self.assertEquals(d[a], "a")
        self.assertEquals(d[b], "b")
