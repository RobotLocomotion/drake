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
        self.assertEqual(str(template), "<TemplateBase {}.BaseTpl>".format(
            _TEST_MODULE))

        # Single arguments.
        template.add_instantiation(int, 1)
        self.assertEqual(template[int], 1)
        self.assertEqual(template.get_instantiation(int), (1, (int,)))
        self.assertEqual(template.get_param_set(1), {(int,)})
        self.assertTrue(template.is_instantiation(1))
        self.assertFalse(template.is_instantiation(10))
        # Duplicate parameters.
        self.assertRaises(
            RuntimeError, lambda: template.add_instantiation(int, 4))

        # Invalid parameters.
        self.assertRaises(RuntimeError, lambda: template[float])
        # New instantiation.
        template.add_instantiation(float, 2)
        self.assertEqual(template[float], 2)

        # Default instantiation.
        self.assertEqual(template[None], 1)
        self.assertEqual(template.get_instantiation(), (1, (int,)))

        # Multiple arguments.
        template.add_instantiation((int, int), 3)
        self.assertEqual(template[int, int], 3)
        # Duplicate instantiation.
        template.add_instantiation((float, float), 1)
        self.assertEqual(template.get_param_set(1), {(int,), (float, float)})
        # Nested getitem indices.
        self.assertEqual(template[(int, int)], 3)
        self.assertEqual(template[[int, int]], 3)

        # List instantiation.
        def instantiation_func(param):
            return 100 + len(param)
        dummy_a = (str,) * 5
        dummy_b = (str,) * 10
        template.add_instantiations(instantiation_func, [dummy_a, dummy_b])
        self.assertEqual(template[dummy_a], 105)
        self.assertEqual(template[dummy_b], 110)

        # Ensure that we can only call this once.
        dummy_c = (str,) * 7
        with self.assertRaises(RuntimeError):
            template.add_instantiations(instantiation_func, [dummy_c])

    def test_class(self):
        template = m.TemplateClass("ClassTpl")
        self.assertEqual(str(template), "<TemplateClass {}.ClassTpl>".format(
            _TEST_MODULE))

        template.add_instantiation(int, DummyA)
        template.add_instantiation(float, DummyB)

        self.assertEqual(template[int], DummyA)
        self.assertEqual(str(DummyA), "<class '{}.ClassTpl[int]'>".format(
            _TEST_MODULE))
        self.assertEqual(template[float], DummyB)
        self.assertEqual(str(DummyB), "<class '{}.ClassTpl[float]'>".format(
            _TEST_MODULE))

    def test_user_class(self):
        test = self

        @m.TemplateClass.define("MyTemplate", param_list=((int,), (float,)))
        def MyTemplate(param):
            T, = param
            # Ensure that we have deferred evaluation.
            test.assertEqual(MyTemplate.param_list, [(int,), (float,)])

            class Impl(object):
                def __init__(self):
                    self.T = T
                    self.mangled_result = self.__mangled_method()

                def __mangled_method(self):
                    # Ensure that that mangled methods are usable.
                    return (T, 10)

            return Impl

        self.assertIsInstance(MyTemplate, m.TemplateClass)
        MyDefault = MyTemplate[None]
        MyInt = MyTemplate[int]
        self.assertEqual(MyDefault, MyInt)
        self.assertEqual(MyInt().T, int)
        MyFloat = MyTemplate[float]
        self.assertEqual(MyFloat().T, float)

        # Test subclass checks.
        class Subclass(MyTemplate[float]):
            pass

        self.assertFalse(MyTemplate.is_instantiation(Subclass))
        self.assertFalse(MyTemplate.is_subclass_of_instantiation(object))
        result = MyTemplate.is_subclass_of_instantiation(Subclass)
        self.assertTrue(result)
        self.assertEqual(result, MyTemplate[float])

        # Test mangling behavior.
        # TODO(eric.couisneau): If we use the name `Impl` for instantiations,
        # then mangling among inherited templated classes will not work as
        # intended. Consider an alternative, if it's ever necessary.
        self.assertEqual(MyInt().mangled_result, (int, 10))
        self.assertEqual(MyInt._original_name, "Impl")
        self.assertTrue(hasattr(MyInt, "_Impl__mangled_method"))
        self.assertEqual(MyFloat._original_name, "Impl")
        self.assertEqual(MyFloat().mangled_result, (float, 10))
        self.assertTrue(hasattr(MyFloat, "_Impl__mangled_method"))

    def test_function(self):
        template = m.TemplateFunction("func")

        template.add_instantiation(int, dummy_a)
        template.add_instantiation(float, dummy_b)

        self.assertEqual(template[int](), 1)
        self.assertEqual(template[float](), 2)
        self.assertEqual(str(template), "<TemplateFunction {}.func>".format(
            _TEST_MODULE))

    def test_method(self):
        DummyC.method = m.TemplateMethod("method", DummyC)
        DummyC.method.add_instantiation(int, DummyC.dummy_c)
        DummyC.method.add_instantiation(float, DummyC.dummy_d)

        self.assertEqual(str(DummyC.method),
                         "<unbound TemplateMethod DummyC.method>")
        self.assertEqual(DummyC.method[int], DummyC.dummy_c)
        self.assertEqual(str(DummyC.method[int]),
                         "<unbound method DummyC.dummy_c>")

        obj = DummyC()
        self.assertTrue(
            str(obj.method).startswith(
                "<bound TemplateMethod DummyC.method of "))
        self.assertTrue(
            str(obj.method[int]).startswith(
                "<bound method DummyC.dummy_c of "))
        self.assertEqual(obj.method[int](), (obj, 3))
        self.assertEqual(DummyC.method[int](obj), (obj, 3))
        self.assertEqual(obj.method[float](), (obj, 4))
        self.assertEqual(DummyC.method[float](obj), (obj, 4))

    def test_get_or_init(self):
        m_test = ModuleType("test_module")

        def get_tpl_cls():
            return m.get_or_init(m_test, "ClassTpl", m.TemplateClass)

        tpl_1 = get_tpl_cls()
        self.assertEqual(str(tpl_1), "<TemplateClass test_module.ClassTpl>")
        self.assertTrue(m_test.ClassTpl is tpl_1)
        tpl_2 = get_tpl_cls()
        self.assertTrue(tpl_1 is tpl_2)

        def get_tpl_method():
            return m.get_or_init(DummyD, "method", m.TemplateMethod, DummyD)

        tpl_1 = get_tpl_method()
        self.assertTrue(tpl_1 is DummyD.method)
        self.assertEqual(str(tpl_1), "<unbound TemplateMethod DummyD.method>")
        tpl_2 = get_tpl_method()
        self.assertTrue(tpl_1 is tpl_2)
