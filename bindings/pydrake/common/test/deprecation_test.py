"""
Tests low-level deprecation API.

See `deprecation_utility_test.py` for a unittest on higher-level API. See
`deprecation_autocomplete_test.py` for an autocompletion test of this API, as
well as an explanation why that test is separate.
"""

import pydrake.common.deprecation as mut  # ruff: isort: skip

import sys
from types import ModuleType
import unittest
import warnings


class TestDeprecation(unittest.TestCase):
    """Tests module shim functionality."""

    def tearDown(self):
        # Remove the module.
        import deprecation_example as example

        # Python versions differ on whether binding an object to a local
        # variable increases the ref count.
        tare = sys.getrefcount(example)
        del sys.modules[example.__name__]
        self.assertEqual(sys.getrefcount(example), tare - 1)

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
            full_message_expected = (
                f"{message_expected} The deprecated code will be removed "
                f"from Drake on or after 2038-01-19."
            )
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
