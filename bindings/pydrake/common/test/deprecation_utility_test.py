"""
Provides a unittest for high-level deprecation testing API
(``catch_drake_warnings``), relevant to deprecations in pybind11 bindings of
C++ API.

For more details, please review the following documentation:

- https://drake.mit.edu/code_review_checklist.html
- https://drake.mit.edu/doxygen_cxx/group__python__bindings.html#PydrakeDeprecation
- https://drake.mit.edu/doxygen_cxx/group__python__bindings.html#PydrakeDoc
"""  # noqa

import unittest

from pydrake.common.test_utilities.deprecation import catch_drake_warnings

import deprecation_example.cc_module as mut_cc


class TestDeprecationExample(unittest.TestCase):
    def test_cc_deprecated_param_init(self):
        with catch_drake_warnings(expected_count=1) as w:
            obj = mut_cc.ExampleCppStruct()
            self.assertIn("Do not use ExampleCppStruct", str(w[0].message))

        self.assertEqual(obj.i, 0)
        self.assertEqual(obj.j, 0)

    def test_cc_py_init_deprecated(self):
        mut_cc.ExampleCppClass()

        with catch_drake_warnings(expected_count=1) as w:
            mut_cc.ExampleCppClass(0)
            self.assertIn("Do not use ExampleCppClass(int)", str(w[0].message))
        with catch_drake_warnings(expected_count=1) as w:
            mut_cc.ExampleCppClass(0.0)
            self.assertIn(
                "Do not use ExampleCppClass(double)", str(w[0].message))

    def test_cc_deprecate_attribute(self):
        obj = mut_cc.ExampleCppClass()

        with catch_drake_warnings(expected_count=2) as w:
            obj.DeprecatedMethod()
            self.assertIn("Do not use DeprecatedMethod", str(w[0].message))
            obj.DeprecatedMethod(int())
            self.assertEqual(str(w[0].message), str(w[1].message))

    def test_cc_wrap_deprecated_for_overload(self):
        obj = mut_cc.ExampleCppClass()

        # Not deprecated.
        obj.overload()

        with catch_drake_warnings(expected_count=1) as w:
            obj.overload(0)
            self.assertIn("Do not use overload(int)", str(w[0].message))

    def test_cc_wrap_deprecated_for_old_kwarg(self):
        obj = mut_cc.ExampleCppClass()

        # Not deprecated.
        obj.FunctionWithArgumentName(1)
        obj.FunctionWithArgumentName(new_name=1)

        with catch_drake_warnings(expected_count=1) as w:
            obj.FunctionWithArgumentName(old_name=1)
            self.assertIn(
                "FunctionWithArgumentName(old_name)", str(w[0].message))
