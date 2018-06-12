from __future__ import print_function

from pydrake.util.deprecation import DrakeDeprecationWarning

import pydoc
import unittest
import sys
from types import ModuleType
import warnings

import pydrake


class TestDeprecation(unittest.TestCase):
    """Tests module shim functionality. """
    def tearDown(self):
        # Remove the module.
        import deprecation_example as mod
        del sys.modules[mod.__name__]
        self.assertEqual(sys.getrefcount(mod), 2)

    def test_module_nominal(self):
        # Test reading and writing as one would do with a normal module.
        import deprecation_example as mod
        self.assertEqual(mod.value, 1)
        mod.value += 10
        self.assertEqual(mod.value, 11)
        mod.something_new = 10
        self.assertEqual(mod.something_new, 10)
        self.assertEqual(mod.__all__, ["value", "sub_module"])
        self.assertTrue(
            str(mod).startswith("<module 'deprecation_example' from"))

    def test_module_import_direct(self):
        # Test an import with direct access.
        import deprecation_example as mod
        # Check submodule access as a non-import.
        self.assertTrue(isinstance(mod.sub_module, str))

    def test_module_import_from_direct(self):
        # Test an import via `from`.
        # N.B. This is imported first because `from {mod} import {var}` could
        # end up resolving `{var}` as a module.
        # @ref https://docs.python.org/3/reference/simple_stmts.html#import
        import deprecation_example
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
        import deprecation_example.import_all
        import deprecation_example
        self.assertIsInstance(deprecation_example.sub_module, str)

    def test_module_import_exec(self):
        # Test `exec` workflow.
        temp = {}
        exec "from deprecation_example import *" in temp
        self.assertIsInstance(temp["sub_module"], str)

    def _check_warning(
            self, item, message_expected, type=DrakeDeprecationWarning):
        self.assertEqual(item.category, type)
        if type == DrakeDeprecationWarning:
            message_expected += DrakeDeprecationWarning.addendum
        self.assertEqual(item.message.message, message_expected)

    def test_member_deprecation(self):
        from deprecation_example import ExampleClass

        def base_deprecation():
            warnings.warn(
                "Non-drake warning", category=DeprecationWarning, stacklevel=2)

        # At this point, no other deprecations should have been thrown, so we
        # will test with the default `once` filter.
        with warnings.catch_warnings(record=True) as w:
            base_deprecation()  # Should not appear.
            obj = ExampleClass()
            # Call each deprecated method / propery repeatedly; it should only
            # warn once per unique line of source code.
            # - Method.
            for _ in range(3):
                method = ExampleClass.deprecated_method
                self.assertEqual(obj.deprecated_method(), 1)
                # The next line will not show a warning.
                self.assertEqual(method(obj), 1)
            self.assertEqual(method.__doc__, ExampleClass.doc_method)
            # - Property.
            for _ in range(3):
                prop = ExampleClass.deprecated_prop
                self.assertEqual(obj.deprecated_prop, 2)
                # The next line will not show a warning.
                self.assertEqual(prop.__get__(obj), 2)
            self.assertEqual(prop.__doc__, ExampleClass.doc_prop)
            # Check warnings.
            self.assertEqual(len(w), 2)
            self._check_warning(w[0], ExampleClass.message_method)
            self._check_warning(w[1], ExampleClass.message_prop)

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
            method = ExampleClass.deprecated_method
            self.assertEqual(len(w), 2)
            self._check_warning(
                w[0], "Non-drake warning", type=DeprecationWarning)
            self._check_warning(w[1], ExampleClass.message_method)

        # Edit the following flags to manually inspect the warnings generated.
        show_warnings = False
        if show_warnings:
            include_base_deprecations = False
            if include_base_deprecations:
                warnings.simplefilter("default", DeprecationWarning)
            else:
                # See above notes for why we have to set this.
                warnings.simplefilter("default", DrakeDeprecationWarning)
            for _ in range(3):
                base_deprecation()
                method = ExampleClass.deprecated_method
                method_extra = ExampleClass.deprecated_method
                prop = ExampleClass.deprecated_prop
                prop_extra = ExampleClass.deprecated_prop
            # N.B. `help(<module>)` is super verbose.
            print("Help text:\n{}".format(
                pydoc.getdoc(pydrake.util.deprecation)))
            # Manually set this back to `once`.
            warnings.simplefilter("ignored", DeprecationWarning)
            warnings.simplefilter("once", DrakeDeprecationWarning)
