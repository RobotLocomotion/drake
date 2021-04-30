from pydrake.common.containers import EqualToDict, namedview, NamedViewBase

import unittest

import numpy as np


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
        keys = list(raw_attempt.keys())
        self.assertFalse(isinstance(keys[0], Item))
        # - Calling `raw()` should provide the desired behavior.
        raw = d.raw()
        keys = list(raw.keys())
        self.assertTrue(isinstance(keys[0], Item))


def is_same_array(a, b):
    # Indicates that two arrays (of the same shape and type) are views into the
    # same memory.
    # See: https://stackoverflow.com/a/55660651/7829525
    return (a.ctypes.data == b.ctypes.data and a.shape == b.shape
            and a.dtype == b.dtype and (a == b).all())


class TestNamedView(unittest.TestCase):
    def test_meta(self):
        a = np.array([1, 2])
        self.assertTrue(is_same_array(a, a))
        self.assertTrue(is_same_array(a, np.asarray(a)))
        self.assertTrue(is_same_array(a[:1], a[:1]))
        b = a.copy()
        self.assertFalse(is_same_array(a, b))

    def test_view_negative(self):
        # Empty.
        with self.assertRaises(RuntimeError) as cm:
            namedview("bad", [])
        self.assertIn("empty", str(cm.exception))
        # Non-str or subview.
        with self.assertRaises(RuntimeError) as cm:
            namedview("bad", [1])
        self.assertIn("`str` or a `namedview`", str(cm.exception))
        # Bad naming of field.
        with self.assertRaises(RuntimeError) as cm:
            namedview("bad", ["a_"])
        self.assertIn("str fields cannot end with `_`", str(cm.exception))
        # Bad naming of subview.
        with self.assertRaises(RuntimeError) as cm:
            namedview("bad", [namedview("a", ["x"])])
        self.assertIn(
            "Nested subview class name must end with `_`", str(cm.exception)
        )
        # Duplicates.
        with self.assertRaises(RuntimeError) as cm:
            namedview("bad", ["a", "a"])
        self.assertIn("Duplicate (non-unique)", str(cm.exception))
        # Mixed nest-via-str and nest-via-subview.
        with self.assertRaises(RuntimeError) as cm:
            namedview("bad", ["a.x", namedview("b_", ["y"])])
        self.assertIn("specified consistently", str(cm.exception))
        # Non-contiguous nest-via-str.
        with self.assertRaises(RuntimeError) as cm:
            namedview("bad", ["a.x", "b", "a.y"])
        self.assertIn("non-contiguous", str(cm.exception))

    def assert_same_field_structure(self, view_cls_a, view_cls_b):
        # Checks that flat and non-flat view of fields are the same.
        self.assertEqual(
            view_cls_a.get_fields(),
            view_cls_b.get_fields(),
        )
        self.assertEqual(
            view_cls_a.get_fields(flat=False),
            view_cls_b.get_fields(flat=False),
        )

    def test_view_basic(self):
        """
        Tests basics of aliasing and meta-programming contracts.
        """
        MyView = namedview("MyView", ["a", "b"])
        self.assertTrue(issubclass(MyView, NamedViewBase))
        self.assertEqual(MyView.__name__, "MyView")
        self.assertEqual(MyView.get_fields(), ("a", "b"))
        value = np.array([1, 2])
        view = MyView(value)
        self.assertEqual(view.a, 1)
        self.assertEqual(view.b, 2)
        self.assertTrue(is_same_array(value, np.asarray(view)))
        view.a = 10
        self.assertEqual(value[0], 10)
        value[1] = 100
        self.assertEqual(view.b, 100)
        view[:] = 3
        np.testing.assert_equal(value, [3, 3])
        self.assertEqual(repr(view), "MyView(a=3, b=3)")
        self.assertEqual(str(view), repr(view))

    def test_view_flexibility(self):
        """
        Ensures that we do not constrain shape (beyond first dim) or dytpe.
        """
        MyView = namedview("MyView", ["a", "b"])
        x_float = np.array([1.0, 2.0])
        view = MyView(x_float)
        self.assertIsInstance(view.a, float)
        self.assertIsInstance(view.b, float)
        # int64
        x_int = np.array([1, 2], dtype=int)
        view = MyView(x_int)
        self.assertIsInstance(view.a, np.int64)
        self.assertIsInstance(view.b, np.int64)

        x_object = np.array([("crazy",), None], dtype=object)
        view = MyView(x_object)
        self.assertIsInstance(view.a, tuple)
        self.assertIs(view.b, None)

        # Multidimensional.
        x_multidim = np.array([
            [1, 2, 3],
            [4, 5, 6],
        ])
        view = MyView(x_multidim)
        np.testing.assert_equal(view.a, [1, 2, 3])
        np.testing.assert_equal(view.b, [4, 5, 6])

    def test_subview(self):
        MyNestedView = namedview(
            "MyNestedView",
            [namedview("a_", ["x", "y"]), "b", namedview("c_", ["qw", "qx"])],
        )
        self.assertEqual(
            MyNestedView.get_fields(),
            ("a.x", "a.y", "b", "c.qw", "c.qx"),
        )
        self.assertEqual(
            MyNestedView.get_fields(flat=False),
            ("a", "b", "c"),
        )
        self.assertEqual(MyNestedView.size, 5)
        # - Test user-friendly constructor.
        MyNestedView2 = namedview(
            "MyNestedView2",
            ["a.x", "a.y", "b", "c.qw", "c.qx"],
        )
        self.assert_same_field_structure(MyNestedView, MyNestedView2)

        value = np.array([1, 2, 3, 4, 5])
        view = MyNestedView(value)
        self.assertEqual(
            (
                view.a.x,
                view.a.y,
                view.b,
                view.c.qw,
                view.c.qx,
            ),
            (
                1,
                2,
                3,
                4,
                5,
            ),
        )
        subview_a_cls = type(view.a)
        self.assertTrue(issubclass(subview_a_cls, NamedViewBase))
        self.assertIs(subview_a_cls, MyNestedView.a_)
        # Weird, but meh.
        self.assertEqual(subview_a_cls.__name__, "a_")
        self.assertEqual(subview_a_cls.get_fields(), ("x", "y"))
        self.assertEqual(
            repr(view),
            "MyNestedView(a.x=1, a.y=2, b=3, c.qw=4, c.qx=5)"
        )
        self.assertEqual(
            repr(view.a),
            "MyNestedView.a_(x=1, y=2)",
        )

        self.assertTrue(is_same_array(value, np.asarray(view)))
        self.assertTrue(is_same_array(value[0:2], np.asarray(view.a)))
        self.assertTrue(is_same_array(value[3:], np.asarray(view.c)))

        # Mutate.
        value[3:] = 0
        np.testing.assert_array_equal(view, [1, 2, 3, 0, 0])
        view.c = [10, 20]
        np.testing.assert_array_equal(value, [1, 2, 3, 10, 20])
        view.c[1] = 100
        np.testing.assert_array_equal(value, [1, 2, 3, 10, 100])

        # Briefly test multi-nesting.
        MyMultiNestedView = namedview(
            "MyMultiNestedView",
            [
                namedview("a_", [
                    namedview("b_", [
                        namedview("c_", ["x", "y"]),
                    ]),
                ])
            ],
        )
        self.assertEqual(MyMultiNestedView.size, 2)
        self.assertEqual(
            MyMultiNestedView.get_fields(),
            ("a.b.c.x", "a.b.c.y"),
        )
        view = MyMultiNestedView([1, 2])
        self.assertEqual(
            repr(view.a.b),
            # TODO(eric.cousineau): Fix qualname via nesting.
            "a_.b_(c.x=1, c.y=2)",
        )
        MyMultiNestedView2 = namedview(
            "MyMultiNestedView2",
            ("a.b.c.x", "a.b.c.y"),
        )
        self.assert_same_field_structure(MyMultiNestedView, MyMultiNestedView2)
