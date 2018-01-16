#!/usr/bin/env python
from __future__ import absolute_import, print_function

import unittest
from types import ModuleType

import pydrake.util.cpp_template as m


class DummyA(object):
    pass


class DummyB(object):
    pass


def dummy_a():
    return 1


def dummy_b():
    return 2


class DummyC(object):
    def dummy_c(self):
        return (self, 3)

    def dummy_d(self):
        return (self, 4)


class DummyD(object):
    pass


class TestCppTemplate(unittest.TestCase):
    def test_base(self):
        tpl = m.TemplateBase("BaseTpl")
        self.assertEquals(str(tpl), "<TemplateBase __main__.BaseTpl>")

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
        # Nested getitem indices.
        self.assertEquals(tpl[(int, int)], 3)
        self.assertEquals(tpl[[int, int]], 3)

        # List instantiation.
        def instantiation_func(param):
            return 100 + len(param)
        dummy_a = (str,) * 5
        dummy_b = (str,) * 10
        tpl.add_instantiations(instantiation_func, [dummy_a, dummy_b])
        self.assertEquals(tpl[dummy_a], 105)
        self.assertEquals(tpl[dummy_b], 110)

    def test_class(self):
        tpl = m.TemplateClass("ClassTpl")

        tpl.add_instantiation(int, DummyA)
        tpl.add_instantiation(float, DummyB)

        self.assertEquals(tpl[int], DummyA)
        self.assertEquals(str(DummyA), "<class '__main__.ClassTpl[int]'>")
        self.assertEquals(tpl[float], DummyB)
        self.assertEquals(str(DummyB), "<class '__main__.ClassTpl[float]'>")

    def test_function(self):
        tpl = m.TemplateFunction("func")

        tpl.add_instantiation(int, dummy_a)
        tpl.add_instantiation(float, dummy_b)

        self.assertEquals(tpl[int](), 1)
        self.assertEquals(tpl[float](), 2)
        self.assertEquals(str(tpl), "<TemplateFunction __main__.func>")

    def test_method(self):
        DummyC.method = m.TemplateMethod("method", DummyC)
        DummyC.method.add_instantiation(int, DummyC.dummy_c)
        DummyC.method.add_instantiation(float, DummyC.dummy_d)

        self.assertEquals(str(DummyC.method),
                          "<unbound TemplateMethod DummyC.method>")
        self.assertEquals(DummyC.method[int], DummyC.dummy_c)
        self.assertEquals(str(DummyC.method[int]),
                          "<unbound method DummyC.dummy_c>")

        obj = DummyC()
        self.assertTrue(
            str(obj.method).startswith(
                "<bound TemplateMethod DummyC.method of "))
        self.assertTrue(
            str(obj.method[int]).startswith(
                "<bound method DummyC.dummy_c of "))
        self.assertEquals(obj.method[int](), (obj, 3))
        self.assertEquals(DummyC.method[int](obj), (obj, 3))
        self.assertEquals(obj.method[float](), (obj, 4))
        self.assertEquals(DummyC.method[float](obj), (obj, 4))

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
            return m.get_or_init(DummyD, "method", m.TemplateMethod, DummyD)

        tpl_1 = get_tpl_method()
        self.assertTrue(tpl_1 is DummyD.method)
        self.assertEquals(str(tpl_1), "<unbound TemplateMethod DummyD.method>")
        tpl_2 = get_tpl_method()
        self.assertTrue(tpl_1 is tpl_2)


if __name__ == '__main__':
    unittest.main()
