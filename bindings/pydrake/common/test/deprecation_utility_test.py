"""
Provides a unittest for high-level deprecation testing API
(``catch_drake_warnings``), relevant to deprecations in pybind11 bindings of
C++ API.

For more details, please review the following documentation:

- https://drake.mit.edu/code_review_checklist.html
- https://drake.mit.edu/doxygen_cxx/group__python__bindings.html#PydrakeDeprecation
- https://drake.mit.edu/doxygen_cxx/group__python__bindings.html#PydrakeDoc
"""  # noqa

import pydrake.common.test_utilities.deprecation as mut  # ruff: isort: skip

import importlib
import os
import unittest

from pydrake.common import Parallelism
import pydrake.common.deprecation as dep

import deprecation_example.cc_module as example  # ruff: isort: skip


class TestDeprecationExample(unittest.TestCase):
    def test_cc_deprecated_param_init(self):
        with mut.catch_drake_warnings(expected_count=1) as w:
            obj = example.ExampleCppStruct()
            self.assertIn("Do not use ExampleCppStruct", str(w[0].message))

        self.assertEqual(obj.i, 0)
        self.assertEqual(obj.j, 0)

    def test_cc_py_init_deprecated(self):
        example.ExampleCppClass()

        with mut.catch_drake_warnings(expected_count=1) as w:
            example.ExampleCppClass(0)
            self.assertIn("Do not use ExampleCppClass(int)", str(w[0].message))
        with mut.catch_drake_warnings(expected_count=1) as w:
            example.ExampleCppClass(0.0)
            self.assertIn(
                "Do not use ExampleCppClass(double)", str(w[0].message)
            )

    def test_cc_deprecate_attribute(self):
        obj = example.ExampleCppClass()

        with mut.catch_drake_warnings(expected_count=2) as w:
            obj.DeprecatedMethod()
            self.assertIn("Do not use DeprecatedMethod", str(w[0].message))
            obj.DeprecatedMethod(int())
            self.assertEqual(str(w[0].message), str(w[1].message))

    def test_cc_wrap_deprecated_for_overload(self):
        obj = example.ExampleCppClass()

        # Not deprecated.
        obj.overload()

        with mut.catch_drake_warnings(expected_count=1) as w:
            obj.overload(0)
            self.assertIn("Do not use overload(int)", str(w[0].message))

    def test_cc_wrap_deprecated_for_parallelism(self):
        obj = example.ExampleCppClass()
        with mut.catch_drake_warnings(expected_count=1) as w:
            obj.ParallelWork(Parallelism.Max())
            self.assertIn("Do not use ParallelWork", str(w[0].message))

    def test_cc_wrap_deprecated_for_old_kwarg(self):
        obj = example.ExampleCppClass()

        # Not deprecated.
        obj.FunctionWithArgumentName(1)
        obj.FunctionWithArgumentName(new_name=1)

        with mut.catch_drake_warnings(expected_count=1) as w:
            obj.FunctionWithArgumentName(old_name=1)
            self.assertIn(
                "FunctionWithArgumentName(old_name)", str(w[0].message)
            )


class TestDeprecationExampleEnv(unittest.TestCase):
    SEVERITY_KEY = "DRAKE_DEPRECATION_RUNTIME_SEVERITY"

    def tearDown(self):
        os.environ.pop(self.SEVERITY_KEY, None)

    def test_ignore_deprecation_warnings(self):
        os.environ[self.SEVERITY_KEY] = "ignore"
        importlib.reload(dep)

        with mut.catch_drake_warnings(expected_count=0):
            with self.subTest("DeprecatedParamInit"):
                obj = example.ExampleCppStruct()

            with self.subTest("py_init_deprecated"):
                example.ExampleCppClass(0)
                example.ExampleCppClass(0.0)

            obj = example.ExampleCppClass()

            with self.subTest("DeprecateAttribute"):
                obj.DeprecatedMethod()
                obj.DeprecatedMethod(int())

            with self.subTest("WrapDeprecated"):
                obj.overload(0)
                obj.ParallelWork(Parallelism.Max())
                obj.FunctionWithArgumentName(old_name=1)

    def test_deprecation_as_errors(self):
        os.environ[self.SEVERITY_KEY] = "error"
        importlib.reload(dep)

        obj = example.ExampleCppClass()

        cases = [
            (lambda: example.ExampleCppStruct(), "Do not use ExampleCppStruct"),
            (lambda: example.ExampleCppClass(0), "Do not use ExampleCppClass(int)"),
            (lambda: example.ExampleCppClass(0.0), "Do not use ExampleCppClass(double)"),
            (lambda: obj.DeprecatedMethod(), "Do not use DeprecatedMethod"),
            (lambda: obj.DeprecatedMethod(int()), "Do not use DeprecatedMethod"),
            (lambda: obj.overload(0), "Do not use overload(int)"),
            (lambda: obj.ParallelWork(Parallelism.Max()), "Do not use ParallelWork"),
            (lambda: obj.FunctionWithArgumentName(old_name=1), "FunctionWithArgumentName(old_name)"),
        ]

        for action, expected_message in cases:
            with self.assertRaises(dep.DrakeDeprecationWarning) as exception_context:
                action()

            self.assertIn(expected_message, str(exception_context.exception))
