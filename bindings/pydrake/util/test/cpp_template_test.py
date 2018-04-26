from __future__ import absolute_import, print_function

import unittest
from types import ModuleType

import pydrake.util.cpp_template as m

_TEST_MODULE = "cpp_template_test"


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
        template = m.TemplateBase("BaseTpl")
        self.assertEquals(str(template), "<TemplateBase {}.BaseTpl>".format(
            _TEST_MODULE))

        # Single arguments.
        template.add_instantiation(int, 1)
        self.assertEquals(template[int], 1)
        self.assertEquals(template.get_instantiation(int), (1, (int,)))
        self.assertEquals(template.get_param_set(1), {(int,)})
        self.assertTrue(m.is_instantiation_of(1, template))
        self.assertFalse(m.is_instantiation_of(10, template))
        # Duplicate parameters.
        self.assertRaises(
            RuntimeError, lambda: template.add_instantiation(int, 4))

        # Invalid parameters.
        self.assertRaises(RuntimeError, lambda: template[float])
        # New instantiation.
        template.add_instantiation(float, 2)
        self.assertEquals(template[float], 2)

        # Default instantiation.
        self.assertEquals(template[None], 1)
        self.assertEquals(template.get_instantiation(), (1, (int,)))

        # Multiple arguments.
        template.add_instantiation((int, int), 3)
        self.assertEquals(template[int, int], 3)
        # Duplicate instantiation.
        template.add_instantiation((float, float), 1)
        self.assertEquals(template.get_param_set(1), {(int,), (float, float)})
        # Nested getitem indices.
        self.assertEquals(template[(int, int)], 3)
        self.assertEquals(template[[int, int]], 3)

        # List instantiation.
        def instantiation_func(template, param):
            self.assertIsInstance(template, m.TemplateBase)
            return 100 + len(param)
        dummy_a = (str,) * 5
        dummy_b = (str,) * 10
        template.add_instantiations(instantiation_func, [dummy_a, dummy_b])
        self.assertEquals(template[dummy_a], 105)
        self.assertEquals(template[dummy_b], 110)

    def test_class(self):
        template = m.TemplateClass("ClassTpl")
        self.assertEquals(str(template), "<TemplateClass {}.ClassTpl>".format(
            _TEST_MODULE))

        template.add_instantiation(int, DummyA)
        template.add_instantiation(float, DummyB)

        self.assertEquals(template[int], DummyA)
        self.assertEquals(str(DummyA), "<class '{}.ClassTpl[int]'>".format(
            _TEST_MODULE))
        self.assertEquals(template[float], DummyB)
        self.assertEquals(str(DummyB), "<class '{}.ClassTpl[float]'>".format(
            _TEST_MODULE))

    def test_user_class(self):

        @m.TemplateClass.define("MyTemplate", param_list=((int,), (float,)))
        def MyTemplate(_, param):
            T, = param

            class MyTemplateInstantiation(object):
                def __init__(self):
                    self.T = T

            return MyTemplateInstantiation

        self.assertIsInstance(MyTemplate, m.TemplateClass)
        MyDefault = MyTemplate[None]
        MyInt = MyTemplate[int]
        self.assertEqual(MyDefault, MyInt)
        self.assertEqual(MyInt().T, int)
        MyFloat = MyTemplate[float]
        self.assertEqual(MyFloat().T, float)

    def test_function(self):
        template = m.TemplateFunction("func")

        template.add_instantiation(int, dummy_a)
        template.add_instantiation(float, dummy_b)

        self.assertEquals(template[int](), 1)
        self.assertEquals(template[float](), 2)
        self.assertEquals(str(template), "<TemplateFunction {}.func>".format(
            _TEST_MODULE))

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
