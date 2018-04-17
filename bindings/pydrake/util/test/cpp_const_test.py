import unittest

import pydrake.util.cpp_const as cpp_const


# Not annotated.
class Basic(object):
    def __init__(self, name):
        self._name = name
        self.value = []

    def get_name(self):
        return self._name

    def set_name(self, name):
        self._name = name

    name = property(get_name, set_name)


# Annotated.
@cpp_const.const_decorated(owned_properties=['_values'])
class Advanced(object):
    def __init__(self):
        self._values = {}

    def add(self, key, value):
        self._values[key] = value

    def get(self, key):
        return self._values[key]

    def get_values(self):
        return self._values

    @cpp_const.mutable_method
    def mutate(self): pass

    def mutate_indirect(self):
        self.mutate()


# Child class.
class AdvancedChild(Advanced):
    def const_safe(self, key):
        return self.get(key)

    def const_unsafe(self, key):
        del self.get_values()[key]

    def mutate_indirect_2(self):
        self.mutate_indirect()


class TestCppConst(unittest.TestCase):
    def _ex(self):
        # Shorthand for testing for errors.
        return self.assertRaises(cpp_const.ConstError)

    def test_list(self):
        # List.
        x = [1, 2, 3, [10]]
        x_const = cpp_const.to_const(x)
        with self._ex():
            x_const[0] = 10
        with self._ex():
            x_const[:] = []
        with self._ex():
            del x_const[0]
        with self._ex():
            x_const.append(10)
        with self._ex():
            x_const.pop()
        # N.B. Access does not propagate.
        for i in x_const:
            self.assertFalse(cpp_const.is_const_test(i))
        self.assertFalse(cpp_const.is_const_test(x_const[3]))

    def test_dict(self):
        # Dictionary.
        d = {"a": 0, "b": 1, "z": [25]}
        d_const = cpp_const.to_const(d)
        self.assertEqual(d_const["a"], 0)
        with self._ex():
            d_const["c"] = 2
        with self._ex():
            d_const.clear()
        # N.B. Access does not implicitly propagate.
        self.assertFalse(cpp_const.is_const_test(d_const["z"]))

    def test_basic(self):
        # Basic class.
        obj = Basic("Tim")
        obj_const = cpp_const.to_const(obj)
        obj.new_attr = "Something"
        self.assertEqual(obj_const.get_name(), "Tim")
        self.assertEqual(obj_const.__dict__["_name"], "Tim")
        with self._ex():
            obj_const.set_name("Bob")
        with self._ex():
            obj_const.name = "Bob"
        with self._ex():
            obj_const._name = "Bob"
        with self._ex():
            obj_const.__dict__["_name"] = "Bob"
        with self._ex():
            obj_const.new_attr = "Something Else"
        # N.B. Access does not implicitly propagate.
        self.assertFalse(cpp_const.is_const_test(obj_const.value))

    def test_advanced(self):
        obj = Advanced()
        obj.add("a", 0)
        obj.add("b", 1)
        obj.add("z", [10])
        obj_const = cpp_const.to_const(obj)
        self.assertTrue(cpp_const.is_const_test(obj_const.get_values()))
        self.assertEqual(obj_const.get("a"), 0)
        with self._ex():
            obj_const.add("c", 2)
        with self._ex():
            obj_const.get_values()["c"] = 2
        with self._ex():
            obj_const.mutate()
        with self._ex():
            obj_const.mutate_indirect()
        # N.B. Access does not implicitly propagate.
        self.assertFalse(cpp_const.is_const_test(obj_const.get("z")))
        self.assertFalse(
            cpp_const.is_const_test(obj_const.__dict__["_values"]))

    def test_child(self):
        obj = AdvancedChild()
        obj.add("a", 0)
        obj_const = cpp_const.to_const(obj)
        self.assertEqual(obj_const.const_safe("a"), 0)
        with self._ex():
            obj_const.const_unsafe("a")
        with self._ex():
            obj_const.mutate_indirect_2()
        with self._ex():
            obj_const._values["c"] = 2
