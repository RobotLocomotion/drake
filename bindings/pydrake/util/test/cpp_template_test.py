#!/usr/bin/env python
from __future__ import absolute_import, print_function

import unittest
from types import ModuleType

import pydrake.util.cpp_template as m


class A(object):
    pass


class B(object):
    pass


def func_a():
    return 1


def func_b():
    return 2


class C(object):
    def func_c(self):
        return (self, 3)

    def func_d(self):
        return (self, 4)


class D(object):
    pass


class TestCppTemplate(unittest.TestCase):
    def test_base(self):
        tpl = m.Template("BaseTpl")
        self.assertEquals(str(tpl), "<Template __main__.BaseTpl>")

        # Single arguments.
        tpl.add_instantiation(int, 1)
        self.assertEquals(tpl[int], 1)
        self.assertEquals(tpl.get_instantiation(int), (1, (int,)))
        self.assertEquals(tpl.get_param_set(1), {(int,)})
        self.assertTrue(m.is_instantiation_of(1, tpl))
        self.assertFalse(m.is_instantiation_of(10, tpl))
        # Duplicate parameters.
        self.assertRaises(RuntimeError, lambda: tpl.add_instantiation(int, 4))

        # Invalid parameters.
        self.assertRaises(RuntimeError, lambda: tpl[float])
        # New instantiation.
        tpl.add_instantiation(float, 2)
        self.assertEquals(tpl[float], 2)

        # Default instantiation.
        self.assertEquals(tpl[None], 1)
        self.assertEquals(tpl.get_instantiation(), (1, (int,)))

        # Multiple arguments.
        tpl.add_instantiation((int, int), 3)
        self.assertEquals(tpl[int, int], 3)
        # Duplicate instantiation.
        tpl.add_instantiation((float, float), 1)
        self.assertEquals(tpl.get_param_set(1), {(int,), (float, float)})

        # List instantiation.
        def instantiation_func(param):
            return 100 + len(param)
        func_a = (str,) * 5
        func_b = (str,) * 10
        tpl.add_instantiations(instantiation_func, [func_a, func_b])
        self.assertEquals(tpl[func_a], 105)
        self.assertEquals(tpl[func_b], 110)

    def test_class(self):
        tpl = m.TemplateClass("ClassTpl")

        tpl.add_instantiation(int, A)
        tpl.add_instantiation(float, B)

        self.assertEquals(tpl[int], A)
        self.assertEquals(str(A), "<class '__main__.ClassTpl[int]'>")
        self.assertEquals(tpl[float], B)
        self.assertEquals(str(B), "<class '__main__.ClassTpl[float]'>")

    def test_function(self):
        tpl = m.TemplateFunction("func")

        tpl.add_instantiation(int, func_a)
        tpl.add_instantiation(float, func_b)

        self.assertEquals(tpl[int](), 1)
        self.assertEquals(tpl[float](), 2)
        self.assertEquals(str(tpl), "<TemplateFunction __main__.func>")

    def test_method(self):
        C.method = m.TemplateMethod("method", C)
        C.method.add_instantiation(int, C.func_c)
        C.method.add_instantiation(float, C.func_d)

        self.assertEquals(str(C.method), "<unbound TemplateMethod C.method>")
        self.assertEquals(C.method[int], C.func_c)
        self.assertEquals(str(C.method[int]), "<unbound method C.func_c>")

        obj = C()
        self.assertTrue(
            str(obj.method).startswith("<bound TemplateMethod C.method of "))
        self.assertTrue(
            str(obj.method[int]).startswith("<bound method C.func_c of "))
        self.assertEquals(obj.method[int](), (obj, 3))
        self.assertEquals(C.method[int](obj), (obj, 3))
        self.assertEquals(obj.method[float](), (obj, 4))
        self.assertEquals(C.method[float](obj), (obj, 4))

    def test_get_or_init(self):
        m_test = ModuleType("test_module")

        def get_tpl_cls():
            return m.get_or_init(m_test, "ClassTpl", m.TemplateClass)

        tpl_1 = get_tpl_cls()
        self.assertEquals(str(tpl_1), "<TemplateClass test_module.ClassTpl>")
        self.assertTrue(m_test.ClassTpl is tpl_1)
        tpl_2 = get_tpl_cls()
        self.assertTrue(tpl_1 is tpl_2)

        def get_tpl_method():
            return m.get_or_init(D, "method", m.TemplateMethod, D)

        tpl_1 = get_tpl_method()
        self.assertTrue(tpl_1 is D.method)
        self.assertEquals(str(tpl_1), "<unbound TemplateMethod D.method>")
        tpl_2 = get_tpl_method()
        self.assertTrue(tpl_1 is tpl_2)


if __name__ == '__main__':
    unittest.main()
