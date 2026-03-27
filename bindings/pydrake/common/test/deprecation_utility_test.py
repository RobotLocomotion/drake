"""
Provides a unittest for high-level deprecation testing API
(``catch_drake_warnings``), relevant to deprecations in pybind11 bindings of
C++ API.

For more details, please review the following documentation:

- https://drake.mit.edu/code_review_checklist.html
- https://drake.mit.edu/doxygen_cxx/group__python__bindings.html#PydrakeDeprecation
- https://drake.mit.edu/doxygen_cxx/group__python__bindings.html#PydrakeDoc
"""  # noqa

import importlib
import os
import unittest

from pydrake.common import Parallelism
import pydrake.common.deprecation as dep
from pydrake.common.test_utilities.deprecation import catch_drake_warnings

import deprecation_example.cc_module as mut_cc  # ruff: isort: skip


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
                "Do not use ExampleCppClass(double)", str(w[0].message)
            )

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

    def test_cc_wrap_deprecated_for_parallelism(self):
        obj = mut_cc.ExampleCppClass()
        with catch_drake_warnings(expected_count=1) as w:
            obj.ParallelWork(Parallelism.Max())
            self.assertIn("Do not use ParallelWork", str(w[0].message))

    def test_cc_wrap_deprecated_for_old_kwarg(self):
        obj = mut_cc.ExampleCppClass()

        # Not deprecated.
        obj.FunctionWithArgumentName(1)
        obj.FunctionWithArgumentName(new_name=1)

        with catch_drake_warnings(expected_count=1) as w:
            obj.FunctionWithArgumentName(old_name=1)
            self.assertIn(
                "FunctionWithArgumentName(old_name)", str(w[0].message)
            )


class TestDeprecationExampleEnv(unittest.TestCase):
    DEPRECATION_IS_ERROR_KEY = "DRAKE_ENV_DEPRECATION_IS_ERROR"
    IGNORE_DEPRECATION_KEY = "DRAKE_ENV_IGNORE_DEPRECATED"

    def tearDown(self):
        os.environ.pop(self.DEPRECATION_IS_ERROR_KEY, None)
        os.environ.pop(self.IGNORE_DEPRECATION_KEY, None)

    def test_ignore_deprecation_warnings(self):
        os.environ[self.IGNORE_DEPRECATION_KEY] = "1"
        importlib.reload(dep)

        with catch_drake_warnings(expected_count=0):
            with self.subTest("DeprecatedParamInit"):
                obj = mut_cc.ExampleCppStruct()

            with self.subTest("py_init_deprecated"):
                mut_cc.ExampleCppClass(0)
                mut_cc.ExampleCppClass(0.0)

            obj = mut_cc.ExampleCppClass()

            with self.subTest("DeprecateAttribute"):
                obj.DeprecatedMethod()
                obj.DeprecatedMethod(int())

            with self.subTest("WrapDeprecated"):
                obj.overload(0)
                obj.ParallelWork(Parallelism.Max())
                obj.FunctionWithArgumentName(old_name=1)

    def test_deprecation_as_errors(self):
        os.environ[self.DEPRECATION_IS_ERROR_KEY] = "1"
        importlib.reload(dep)

        obj = mut_cc.ExampleCppClass()

        actions = [
            lambda: mut_cc.ExampleCppStruct(),
            lambda: mut_cc.ExampleCppClass(0),
            lambda: mut_cc.ExampleCppClass(0.0),
            lambda: obj.DeprecatedMethod(),
            lambda: obj.DeprecatedMethod(int()),
            lambda: obj.overload(0),
            lambda: obj.ParallelWork(Parallelism.Max()),
            lambda: obj.FunctionWithArgumentName(old_name=1),
        ]
        msgs = [
            "Do not use ExampleCppStruct",
            "Do not use ExampleCppClass(int)",
            "Do not use ExampleCppClass(double)",
            "Do not use DeprecatedMethod",
            "Do not use DeprecatedMethod",
            "Do not use overload(int)",
            "Do not use ParallelWork",
            "FunctionWithArgumentName(old_name)",
        ]

        for action, msg in zip(actions, msgs):
            with self.assertRaises(dep.DrakeDeprecationWarning) as exc:
                action()

            exc_msg = str(exc.exception)
            self.assertIn(msg, exc_msg)
