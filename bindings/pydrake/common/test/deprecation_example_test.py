"""
Provides an easy-to-use template (pattern matching) for C++/Python API
deprecation.

For more details, please review the following documentation:

- https://drake.mit.edu/code_review_checklist.html
- https://drake.mit.edu/doxygen_cxx/group__python__bindings.html#PydrakeDeprecation
- https://drake.mit.edu/doxygen_cxx/group__python__bindings.html#PydrakeDoc
"""  # noqa

import unittest

from pydrake.common.test_utilities.deprecation import catch_drake_warnings

import deprecation_example as mut
import deprecation_example.cc_module as mut_cc


class TestDeprecationExample(unittest.TestCase):
    def test_example_cpp_struct(self):
        with catch_drake_warnings(expected_count=1) as w:
            obj = mut_cc.ExampleCppStruct()
        self.assertIn("Do not use ExampleCppStruct", str(w[0].message))

        self.assertEqual(obj.i, 0)
        self.assertEqual(obj.j, 0)

    def test_example_cpp_class_ctors(self):
        mut_cc.ExampleCppClass()

        with catch_drake_warnings(expected_count=1) as w:
            mut_cc.ExampleCppClass(0)
        self.assertIn("Do not use ExampleCppClass(int)", str(w[0].message))
        with catch_drake_warnings(expected_count=1) as w:
            mut_cc.ExampleCppClass(0.0)
        self.assertIn("Do not use ExampleCppClass(double)", str(w[0].message))

    def test_example_cpp_class_methods(self):
        obj = mut_cc.ExampleCppClass()

        with catch_drake_warnings(expected_count=1) as w:
            obj.DeprecatedMethod()
        self.assertIn("Do not use DeprecatedMethod()", str(w[0].message))

        # Not deprecated.
        obj.overload()

        with catch_drake_warnings(expected_count=1) as w:
            obj.overload(0)
        self.assertIn("Do not use overload(int)", str(w[0].message))

    def test_example_cpp_class_properties(self):
        obj = mut_cc.ExampleCppClass()

        self.assertEqual(obj.prop, 0)

        with catch_drake_warnings(expected_count=1) as w:
            self.assertEqual(obj.deprecated_aliased_prop, 0)
        self.assertIn("Do not use deprecated_aliased_prop", str(w[0].message))
