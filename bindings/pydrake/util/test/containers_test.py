from pydrake.util.containers import EqualToDict

import unittest


class Comparison(object):
    def __init__(self, lhs, rhs):
        self.lhs = lhs
        self.rhs = rhs

    def __nonzero__(self):
        raise ValueError("Should not be called")


class Item(object):
    equal_to_called = False

    def __init__(self, value):
        self.value = value

    def __hash__(self):
        return hash(self.value)

    def __eq__(self, other):
        # Non-boolean return value.
        return Comparison(self.value, other.value)

    def EqualTo(self, other):
        Item.equal_to_called = True
        return hash(self) == hash(other)


# Globals for testing.
a = Item(1)
b = Item(2)


class TestEqualToDict(unittest.TestCase):
    def test_normal_dict(self):
        d = {a: "a", b: "b"}
        # TODO(eric.cousineau): Figure out how to reproduce failure when `dict`
        # attempts to use `__eq__`, similar to what happens when using
        # `Polynomial` as a key in a dictionary.
        self.assertEqual(d[a], "a")
        with self.assertRaises(ValueError):
            value = bool(a == b)

    def test_equal_to_dict(self):
        d = EqualToDict({a: "a", b: "b"})
        # Ensure that we call `EqualTo`.
        self.assertFalse(Item.equal_to_called)
        self.assertEqual(d[a], "a")
        self.assertTrue(Item.equal_to_called)

        self.assertEqual(d[b], "b")
        self.assertTrue(a in d)

        # Ensure hash collision does not occur.
        self.assertEqual(hash(a.value), hash(a))
        self.assertFalse(a.value in d)

        # Obtaining the original representation (e.g. for `pybind11`):
        # - Constructing using `dict` will not be what is desired; the keys at
        # present are not directly convertible, thus would create an error.
        # N.B. At present, this behavior may not be overridable via Python, as
        # copying is done via `dict.update`, which has a special case for
        # `dict`-inheriting types which does not have any hooks for key
        # transformations.
        raw_attempt = dict(d)
        self.assertFalse(isinstance(raw_attempt.keys()[0], Item))
        # - Calling `raw()` should provide the desired behavior.
        raw = d.raw()
        self.assertTrue(isinstance(raw.keys()[0], Item))
