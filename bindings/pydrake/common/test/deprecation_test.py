"""
Tests low-level deprecation API.

See `deprecation_utility_test.py` for a unittest on higher-level API. See
`deprecation_autocomplete_test.py` for an autocompletion test of this API, as
well as an explanation why that test is separate.
"""

import pydrake.common.deprecation as mut  # ruff: isort: skip
import deprecation_example.cc_module as cc_example  # ruff: isort: skip

import importlib
import os
import sys
from types import ModuleType
import unittest
import warnings

from pydrake.common import Parallelism


def _get_full_message(partial_message):
    return (
        f"{partial_message} The deprecated code will be removed "
        f"from Drake on or after 2038-01-19."
    )


class TestDeprecation(unittest.TestCase):
    """Tests module shim functionality."""

    SEVERITY_KEY = "DRAKE_DEPRECATION_RUNTIME_SEVERITY"

    def _set_severity_env(self, value: str):
        os.environ[self.SEVERITY_KEY] = value
        importlib.reload(mut)

    def tearDown(self):
        # Remove the module.
        import deprecation_example as example

        # Python versions differ on whether binding an object to a local
        # variable increases the ref count.
        tare = sys.getrefcount(example)
        del sys.modules[example.__name__]
        self.assertEqual(sys.getrefcount(example), tare - 1)

        # Undo the _set_severity_env
        os.environ.pop(self.SEVERITY_KEY, None)
        importlib.reload(mut)

    def test_module_nominal(self):
        # Test reading and writing as one would do with a normal module.
        import deprecation_example as example

        self.assertEqual(example.value, 1)
        example.value += 10
        self.assertEqual(example.value, 11)
        example.something_new = 10
        self.assertEqual(example.something_new, 10)
        self.assertEqual(example.__all__, ["value", "sub_module"])
        self.assertTrue(
            str(example).startswith("<module 'deprecation_example' from")
        )

    def test_module_import_direct(self):
        # Test an import with direct access.
        import deprecation_example as example

        # Check submodule access as a non-import.
        self.assertTrue(isinstance(example.sub_module, str))

    def test_module_import_from_direct(self):
        # Test an import via `from`.
        # N.B. This is imported first because `from {mod} import {var}` could
        # end up resolving `{var}` as a module.
        # @ref https://docs.python.org/3/reference/simple_stmts.html#import
        import deprecation_example  # noqa: F401 (unused-import)

        # Test submodule behavior.
        # `from_direct` should use the `getattr` overload.
        from deprecation_example import sub_module as sub

        self.assertTrue(isinstance(sub, str))
        # A nominal import should skip `getattr`, and only use the module
        # loader.
        import deprecation_example.sub_module as new_sub

        self.assertTrue(isinstance(new_sub, ModuleType))

    def test_module_import_from_all(self):
        # N.B. This is done in another module because `from x import *` is not
        # well supported for `exec`.
        import deprecation_example.import_all  # ruff: isort: skip
        import deprecation_example  # ruff: isort: skip

        self.assertIsInstance(deprecation_example.sub_module, str)

    def test_module_import_exec(self):
        # Test `exec` workflow.
        temp = {}
        exec("from deprecation_example import *", temp, temp)
        self.assertIsInstance(temp["sub_module"], str)

    def _check_warning(self, item, message_expected, check_full=True):
        self.assertEqual(item.category, mut.DrakeDeprecationWarning)
        if check_full:
            full_message_expected = _get_full_message(message_expected)
            self.assertEqual(full_message_expected, str(item.message))
        else:
            self.assertIn(message_expected, str(item.message))

    def test_python_deprecation_usages(self):
        """
        Tests low-level deprecation API for members.

        Please see `deprecation_utility_test.py` for a unittest on
        higher-level API.
        """
        import deprecation_example as example

        def base_deprecation():
            warnings.warn(
                "Non-drake warning", category=DeprecationWarning, stacklevel=2
            )

        # At this point, no other deprecations should have been thrown, so we
        # will test with the default `once` filter.
        with warnings.catch_warnings(record=True) as w:
            # Recreate warning environment.
            warnings.simplefilter("ignore", DeprecationWarning)
            warnings.simplefilter("once", mut.DrakeDeprecationWarning)
            # TODO(eric.cousineau): Also different behavior here...
            # Is `unittest` setting a non-standard warning filter???
            base_deprecation()  # Should not appear.
            obj = example.ExampleClass()
            # Call each deprecated method / property repeatedly; it should only
            # warn once per unique line of source code.
            # - Method.
            for _ in range(3):
                method = example.ExampleClass.deprecated_method
                self.assertEqual(obj.deprecated_method(), 1)
                # The next line will not show a warning.
                self.assertEqual(method(obj), 1)
            self.assertEqual(method.__doc__, example.ExampleClass.doc_method)
            # - Property.
            for _ in range(3):
                prop = example.ExampleClass.deprecated_prop
                self.assertEqual(obj.deprecated_prop, 2)
                # The next line will not show a warning.
                self.assertEqual(prop.__get__(obj), 2)
            self.assertEqual(prop.__doc__, example.ExampleClass.doc_prop)
            # - Free function.
            for _ in range(3):
                x = 50
                y = example.deprecated_func(x)
                self.assertEqual(y, 2 * x)
            # Check warnings.
            self.assertEqual(len(w), 3, "\n".join(map(str, w)))
            self._check_warning(w[0], example.ExampleClass.message_method)
            self._check_warning(w[1], example.ExampleClass.message_prop)
            self._check_warning(w[2], example.message_func)

        # Because `once` uses a (somehow opaque) registry (not accessible via
        # `warnings.once*registry` or `_warnings.once_registry`), we must
        # change the filter to test behavior. `reload`ing `warnings` and/or
        # `_warnings` also does not work.
        # See caveat here with `catch_warnings`:
        # https://docs.python.org/2/library/warnings.html#testing-warnings

        # Enable deprecation warnings as well.
        with warnings.catch_warnings(record=True) as w:
            # N.B. This also overrides `DrakeDeprecationWarning` settings
            # because of ordering. We can redefine a filter for
            # `DrakeDeprecationWarning` if so desired.
            warnings.simplefilter("default", DeprecationWarning)
            base_deprecation()
            method = example.ExampleClass.deprecated_method
            self.assertEqual(len(w), 2)
            self.assertEqual(w[0].category, DeprecationWarning)
            self.assertEqual(str(w[0].message), "Non-drake warning")
            self._check_warning(w[1], example.ExampleClass.message_method)

        # Edit the following flags to manually inspect the warnings generated.
        show_warnings = False
        if show_warnings:
            include_base_deprecations = False
            if include_base_deprecations:
                warnings.simplefilter("default", DeprecationWarning)
            else:
                # See above notes for why we have to set this.
                warnings.simplefilter("default", mut.DrakeDeprecationWarning)
            for _ in range(3):
                base_deprecation()
                example.ExampleClass.deprecated_method
                example.ExampleClass.deprecated_method
                example.ExampleClass.deprecated_prop
                example.ExampleClass.deprecated_prop
            # Manually set this back to `once`.
            warnings.simplefilter("ignore", DeprecationWarning)
            warnings.simplefilter("once", mut.DrakeDeprecationWarning)

    def test_deprecated_callable(self):
        """
        Tests low-level deprecation API for callables.

        Please see `deprecation_utility_test.py` for a unittest on
        higher-level API.
        """
        import deprecation_example.cc_module as example

        # Spoof module name.
        var_dict = dict(__name__="fake_module")
        mut._forward_callables_as_deprecated(
            var_dict, example, date="2038-01-19"
        )
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("once", mut.DrakeDeprecationWarning)
            obj = var_dict["ExampleCppClass"]()
            self.assertIsInstance(obj, example.ExampleCppClass)
            message_expected = (
                "Please use ``deprecation_example.cc_module.ExampleCppClass`` "
                "instead of ``fake_module.ExampleCppClass``."
            )
            self.assertEqual(len(w), 1)
            self._check_warning(w[0], message_expected)

    def test_invalid_deprecation_env(self):
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            self._set_severity_env("bad_value")

        self.assertEqual(len(w), 1)
        self.assertEqual(
            str(w[0].message),
            "DRAKE_DEPRECATION_RUNTIME_SEVERITY is set to an unrecognized"
            " value 'bad_value'. Deprecation messages will be emitted"
            " as warnings.",
        )

    def test_ignore_deprecation_warnings(self):
        self._set_severity_env("ignore")
        import deprecation_example as example

        with warnings.catch_warnings(record=True) as w:
            obj = example.ExampleClass()
            obj.deprecated_method()
            obj.deprecated_prop
            example.deprecated_func(50)

            self.assertEqual(len(w), 0, "\n".join(map(str, w)))

    def test_deprecation_as_errors(self):
        self._set_severity_env("error")
        import deprecation_example as example

        obj = example.ExampleClass()

        cases = {
            example.ExampleClass.message_method: lambda: (
                obj.deprecated_method()
            ),
            example.ExampleClass.message_prop: lambda: obj.deprecated_prop,
            example.message_func: lambda: example.deprecated_func(50),
        }

        for message, action in cases.items():
            with self.assertRaises(
                mut.DrakeDeprecationWarning
            ) as exception_context:
                action()

            expected_message = _get_full_message(message)
            actual_message = str(exception_context.exception)
            self.assertEqual(actual_message, expected_message)

    def test_ignore_deprecation_warnings_bindings(self):
        self._set_severity_env("ignore")

        with warnings.catch_warnings(record=True) as w:
            with self.subTest("DeprecatedParamInit"):
                obj = cc_example.ExampleCppStruct()

            with self.subTest("py_init_deprecated"):
                cc_example.ExampleCppClass(0)
                cc_example.ExampleCppClass(0.0)

            obj = cc_example.ExampleCppClass()

            with self.subTest("DeprecateAttribute"):
                obj.DeprecatedMethod()
                obj.DeprecatedMethod(int())

            with self.subTest("WrapDeprecated"):
                obj.overload(0)
                obj.ParallelWork(Parallelism.Max())
                obj.FunctionWithArgumentName(old_name=1)

            self.assertEqual(len(w), 0, "\n".join(map(str, w)))

    def test_deprecation_as_errors_bindings(self):
        self._set_severity_env("error")

        obj = cc_example.ExampleCppClass()

        cases = [
            (
                lambda: cc_example.ExampleCppStruct(),
                "Do not use ExampleCppStruct",
            ),
            (
                lambda: cc_example.ExampleCppClass(0),
                "Do not use ExampleCppClass(int)",
            ),
            (
                lambda: cc_example.ExampleCppClass(0.0),
                "Do not use ExampleCppClass(double)",
            ),
            (lambda: obj.DeprecatedMethod(), "Do not use DeprecatedMethod"),
            (
                lambda: obj.DeprecatedMethod(int()),
                "Do not use DeprecatedMethod",
            ),
            (lambda: obj.overload(0), "Do not use overload(int)"),
            (
                lambda: obj.ParallelWork(Parallelism.Max()),
                "Do not use ParallelWork",
            ),
            (
                lambda: obj.FunctionWithArgumentName(old_name=1),
                "FunctionWithArgumentName(old_name)",
            ),
        ]

        for action, expected_message in cases:
            with self.assertRaises(
                mut.DrakeDeprecationWarning
            ) as exception_context:
                action()

            self.assertIn(expected_message, str(exception_context.exception))
