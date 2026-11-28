from copy import copy as shallow_copy
import unittest

import numpy as np

from pydrake.common.containers import EqualToDict, NamedViewBase, namedview
from pydrake.symbolic import Variable


class Comparison:
    def __init__(self, lhs, rhs):
        self.lhs = lhs
        self.rhs = rhs

    def __bool__(self):
        raise ValueError("Should not be called")

    __nonzero__ = __bool__


class Item:
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


class CollidingItem:
    # Distinct keys with identical hashes, to force dict collision handling.
    def __init__(self, value):
        self.value = value

    def __hash__(self):
        return 0

    def __eq__(self, other):
        # Non-boolean return value.
        return Comparison(self.value, other.value)

    def EqualTo(self, other):
        return self.value == other.value


class TestEqualToDict(unittest.TestCase):
    def setUp(self):
        Item.equal_to_called = False

    def test_normal_dict(self):
        d = {a: "a", b: "b"}
        # TODO(eric.cousineau): Figure out how to reproduce failure when `dict`
        # attempts to use `__eq__`, similar to what happens when using
        # `Polynomial` as a key in a dictionary.
        self.assertEqual(d[a], "a")
        with self.assertRaises(ValueError):
            bool(a == b)

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
        # - Copying using `dict` goes through our iteration, so it also
        # provides the original keys.
        raw_attempt = dict(d)
        keys = list(raw_attempt.keys())
        self.assertTrue(isinstance(keys[0], Item))
        # - Calling `raw()` should provide the desired behavior.
        raw = d.raw()
        keys = list(raw.keys())
        self.assertTrue(isinstance(keys[0], Item))

    def test_equal_to_dict_methods(self):
        # The methods of `dict` that take or return keys must use the same
        # `EqualTo` semantics (and the original key objects) as indexing.
        c = Item(3)
        d = EqualToDict({a: "a"})
        self.assertEqual(d.get(a), "a")
        self.assertIsNone(d.get(b))
        self.assertEqual(d.get(b, "default"), "default")

        # Iteration yields the original keys, like `keys()`.
        self.assertIs(next(iter(d)), a)
        self.assertTrue(all(isinstance(x, Item) for x in d))

        self.assertEqual(d.setdefault(a, "x"), "a")
        self.assertEqual(d.setdefault(b, "b"), "b")
        self.assertEqual(len(d), 2)

        d.update({a: "A"})
        d.update([(c, "c")])
        self.assertEqual(len(d), 3)
        self.assertEqual(d[a], "A")
        self.assertEqual(d[c], "c")

        self.assertEqual(d.pop(c), "c")
        self.assertEqual(d.pop(c, "gone"), "gone")
        with self.assertRaises(KeyError):
            d.pop(c)
        self.assertFalse(c in d)

        key, value = d.popitem()
        # N.B. popitem() is LIFO.
        self.assertIs(key, b)
        self.assertEqual(value, "b")
        self.assertEqual(len(d), 1)

        d |= {b: "B"}
        self.assertEqual(d[b], "B")

    def test_equal_to_dict_equal_but_distinct_keys(self):
        # Keys that are equal but not identical must not be compared with
        # `__eq__` (which does not return a bool) by update() or the
        # constructor.
        a_copy = Item(1)
        d = EqualToDict()
        d.update([(a, "a"), (a_copy, "a_copy")])
        self.assertEqual(len(d), 1)
        self.assertEqual(d[a], "a_copy")
        d = EqualToDict([(a, "a"), (a_copy, "a_copy")])
        self.assertEqual(len(d), 1)

    def test_equal_to_dict_symbolic_variable(self):
        # Two distinct Python objects for the same `Variable`, as with
        # `X[0, 1]` and `X[1, 0]` of `NewSymmetricContinuousVariables()`.
        x = Variable("x")
        x_copy = shallow_copy(x)
        self.assertIsNot(x, x_copy)
        self.assertTrue(x.EqualTo(x_copy))
        d = EqualToDict([(x, 1.0), (x_copy, 2.0)])
        self.assertEqual(len(d), 1)
        self.assertEqual(d[x], 2.0)
        d = EqualToDict()
        d.update([(x, 1.0), (x_copy, 2.0)])
        self.assertEqual(len(d), 1)
        self.assertEqual(d[x_copy], 2.0)

    def test_equal_to_dict_update_forms(self):
        c = Item(3)
        d = EqualToDict()
        d.update({a: "a"})
        self.assertEqual(d[a], "a")
        # `update(<EqualToDict>)`, with a distinct-but-equal key.
        other = EqualToDict({Item(1): "A", c: "c"})
        d.update(other)
        self.assertEqual(len(d), 2)
        self.assertEqual(d[a], "A")
        # Updating from itself is a no-op.
        d.update(d)
        self.assertEqual(len(d), 2)
        # `|=` with an equal-but-distinct key replaces the value.
        d |= {Item(1): "AA"}
        self.assertEqual(len(d), 2)
        self.assertEqual(d[a], "AA")

    def test_equal_to_dict_update_kwargs(self):
        # Keyword arguments are accepted (the keys are `str`, which has no
        # `EqualTo`, so only insertion into a dict without that key is
        # meaningful).
        d = EqualToDict({a: "a"})
        d.update(x=1)
        self.assertEqual(len(d), 2)
        self.assertTrue(any(isinstance(k, str) for k in d))

    def test_equal_to_dict_popitem_empty(self):
        with self.assertRaises(KeyError):
            EqualToDict().popitem()

    def test_equal_to_dict_keyerror_payload(self):
        d = EqualToDict({a: "a"})
        for op in (d.pop, d.__getitem__, d.__delitem__):
            with self.assertRaises(KeyError) as cm:
                op(b)
            self.assertIs(cm.exception.args[0], b)

    def test_equal_to_dict_copy_and_views(self):
        d = EqualToDict({a: "a", b: "b"})
        for dut in (d.copy(), shallow_copy(d), EqualToDict(d)):
            self.assertIsInstance(dut, EqualToDict)
            self.assertIsNot(dut, d)
            self.assertEqual(dut[Item(1)], "a")
            self.assertEqual(len(dut), 2)
        # Mutating the copy leaves the original alone.
        dut = d.copy()
        del dut[a]
        self.assertTrue(a in d)
        # `reversed` and `repr` use the original keys.
        self.assertEqual([x.value for x in reversed(d)], [2, 1])
        self.assertEqual(list(reversed(d)), list(d)[::-1])
        self.assertNotIn("Proxy", repr(d))
        self.assertTrue(repr(d).startswith("EqualToDict({"))
        self.assertIn(repr(a), repr(d))

    def test_equal_to_dict_or(self):
        d = EqualToDict({a: "a"})
        # `|` returns an `EqualToDict` and leaves the operands alone.
        result = d | {Item(1): "A", b: "b"}
        self.assertIsInstance(result, EqualToDict)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[a], "A")
        self.assertEqual(d[a], "a")
        self.assertEqual(len(d), 1)
        result = {Item(1): "X", Item(5): "e"} | d
        self.assertIsInstance(result, EqualToDict)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[a], "a")
        with self.assertRaises(TypeError):
            d | 3

    def test_equal_to_dict_hash_collisions(self):
        k1 = CollidingItem(1)
        k2 = CollidingItem(2)
        # N.B. A dict literal would itself compare the colliding keys.
        d = EqualToDict([(k1, 1), (k2, 2)])
        self.assertEqual(len(d), 2)
        self.assertEqual(d[k2], 2)
        # `copy()` and `EqualToDict(d)` stay safe on colliding keys.
        self.assertEqual(d.copy()[CollidingItem(2)], 2)
        self.assertEqual(EqualToDict(d)[CollidingItem(1)], 1)
        # A plain `dict` of colliding symbolic-like keys uses their own
        # `__eq__`; that is the documented caveat for `dict(d)` and `raw()`.
        with self.assertRaises(ValueError):
            dict(d)
        with self.assertRaises(ValueError):
            d.raw()


def is_same_array(a, b):
    # Indicates that two arrays (of the same shape and type) are views into the
    # same memory.
    # See: https://stackoverflow.com/a/55660651/7829525
    return (
        a.ctypes.data == b.ctypes.data
        and a.shape == b.shape
        and a.dtype == b.dtype
        and (a == b).all()
    )


class TestNamedView(unittest.TestCase):
    def test_meta(self):
        a = np.array([1, 2])
        self.assertTrue(is_same_array(a, a))
        self.assertTrue(is_same_array(a, a[:]))
        self.assertTrue(is_same_array(a, np.asarray(a)))
        b = a.copy()
        self.assertFalse(is_same_array(a, b))
        # Show that `np.array()` by default copies the data regardless, and
        # thus is not the same as `np.asarray()`.
        c = np.array(a)
        self.assertFalse(is_same_array(a, c))
        d = np.array(a, copy=False)
        self.assertTrue(is_same_array(a, d))

    def test_array(self):
        MyView = namedview("MyView", ["a", "b"])
        self.assertTrue(issubclass(MyView, NamedViewBase))
        self.assertEqual(MyView.__name__, "MyView")
        self.assertEqual(MyView.get_fields(), ("a", "b"))
        value = np.array([1.0, 2.0])
        view = MyView(value)
        self.assertEqual(view.a, 1.0)
        self.assertEqual(view.b, 2.0)
        self.assertTrue(is_same_array(value, np.asarray(view)))
        self.assertTrue(is_same_array(value, view[:]))
        view.a = 10.0
        self.assertEqual(value[0], 10.0)
        value[1] = 100.0
        self.assertEqual(view.b, 100.0)
        view[:] = 3.0
        np.testing.assert_equal(value, [3.0, 3.0])
        self.assertEqual(str(view), "MyView(a=3.0, b=3.0)")
        self.assertEqual(repr(view), f"<MyView(a={view.a!r}, b={view.b!r})>")
        with self.assertRaisesRegex(AttributeError, ".*('a', 'b').*"):
            view.c = 42

    def test_array_dtype(self):
        MyView = namedview("MyView", ["a", "b"])
        value = np.array([1.0, 2.0])
        view = MyView(value)
        converted = np.asarray(view, dtype=np.float32)
        self.assertEqual(converted.dtype, np.float32)
        np.testing.assert_equal(converted, value)
        self.assertFalse(np.shares_memory(converted, value))
        self.assertTrue(
            is_same_array(np.asarray(view, dtype=value.dtype), value)
        )

    def test_array_copy(self):
        MyView = namedview("MyView", ["a", "b"])
        value = np.array([1.0, 2.0, 3.0, 4.0])[::2]
        view = MyView(value)
        alias = np.array(view, copy=False)
        self.assertTrue(is_same_array(alias, value))
        alias[0] = 5.0
        self.assertEqual(view.a, 5.0)
        for copied in (np.array(view), np.array(view, copy=True)):
            np.testing.assert_equal(copied, value)
            self.assertFalse(np.shares_memory(copied, value))
            copied[0] = 10.0
            self.assertEqual(view.a, 5.0)

    def test_array_copy_requires_allocation(self):
        MyView = namedview("MyView", ["a", "b"])
        # Match NumPy's copy policy for the backing value, including the
        # stricter copy=False behavior introduced in NumPy 2.
        for value, dtype in (
            ([1.0, 2.0], None),
            (np.array([1.0, 2.0]), np.float32),
        ):
            view = MyView(value)
            for copy in (False, True):
                try:
                    expected = np.array(value, dtype=dtype, copy=copy)
                    expect_raise = False
                except ValueError:
                    expect_raise = True
                with self.subTest(value=value, dtype=dtype, copy=copy):
                    if expect_raise:
                        with self.assertRaises(ValueError):
                            np.array(view, dtype=dtype, copy=copy)
                    else:
                        actual = np.array(view, dtype=dtype, copy=copy)
                        self.assertEqual(actual.dtype, expected.dtype)
                        np.testing.assert_equal(actual, expected)

    def test_Zero(self):
        MyView = namedview("MyView", ["a", "b"])
        view = MyView.Zero()
        self.assertEqual(len(view), 2)
        self.assertEqual(view.a, 0)
        self.assertEqual(view.b, 0)

    def test_name_sanitation(self):
        MyView = namedview(
            "MyView", ["$xyz_base", "iiwa::iiwa", "no spaces", "2vär"]
        )
        self.assertEqual(
            MyView.get_fields(),
            ("_xyz_base", "iiwa_iiwa", "no_spaces", "_2vär"),
        )
        view = MyView.Zero()
        view._xyz_base = 3
        view.iiwa_iiwa = 4
        view.no_spaces = 5
        view._2vär = 6
        np.testing.assert_equal(view[:], [3, 4, 5, 6])

        MyView = namedview(
            "MyView", ["$xyz_base", "iiwa::iiwa"], sanitize_field_names=False
        )
        self.assertEqual(MyView.get_fields(), ("$xyz_base", "iiwa::iiwa"))

    def test_uniqueness(self):
        with self.assertRaisesRegex(AssertionError, ".*must be unique.*"):
            namedview("MyView", ["a", "a"])
        with self.assertRaisesRegex(AssertionError, ".*must be unique.*"):
            namedview("MyView", ["a_a", "a__a"])
