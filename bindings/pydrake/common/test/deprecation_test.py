"""
Tests low-level deprecation API.

Please see `deprecation_utility_test.py` for a unittest on higher-level API.
"""

import rlcompleter
import sys
from types import ModuleType
import unittest
import warnings

from pydrake.common.deprecation import (
    DrakeDeprecationWarning, _forward_callables_as_deprecated)


def get_completion_suffixes(namespace, prefix, max_count=1000):
    # Gets all completions for a given namespace and prefix, stripping the
    # prefix from the results.
    completer = rlcompleter.Completer(namespace)
    suffixes = []
    for i in range(max_count):
        candidate = completer.complete(prefix, i)
        if candidate is None:
            break
        assert candidate.startswith(prefix), (prefix, candidate)
        suffixes.append(candidate[len(prefix):])
    else:
        raise RuntimeError("Exceeded max count!")
    return suffixes


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
        exec("from deprecation_example import *", temp, temp)
        self.assertIsInstance(temp["sub_module"], str)

    def test_module_autocomplete(self):
        # Ensure that we can autocomplete with our example module.
        # Without `__dir__` being implemented, it'll only return `install` as a
        # non-private autocomplete candidate.
        import deprecation_example
        suffixes = get_completion_suffixes(
            locals(), prefix="deprecation_example.")
        suffixes_expected = [
            # Injection from `Completer.attr_matches`, via `get_class_members`.
            "__class__(",
            "__delattr__(",
            "__dict__",
            "__dir__(",
            "__doc__",
            "__format__(",
            "__getattr__(",
            "__getattribute__(",
            "__hash__(",
            "__init__(",
            "__module__",
            "__new__(",
            "__reduce__(",
            "__reduce_ex__(",
            "__repr__(",
            "__setattr__(",
            "__sizeof__(",
            "__str__(",
            "__subclasshook__(",
            "__weakref__",
            "_install(",
            # Intended completions via `__all__`.
            "sub_module",
            "value",
        ]
        suffixes_expected += [
            "__ge__(",
            "__eq__(",
            "__le__(",
            "__lt__(",
            "__gt__(",
            "__ne__(",
        ]
        if hasattr(deprecation_example, "__init_subclass__"):
            suffixes_expected.append("__init_subclass__(")
        # For Bionic Python3, the behavior of autocompletion seems to
        # constrain behavior depending on underscore prefixes.
        if "__init__(" not in suffixes:
            under = get_completion_suffixes(
                locals(), prefix="deprecation_example._")
            suffixes += ["_" + s for s in under]
            dunder = get_completion_suffixes(
                locals(), prefix="deprecation_example.__")
            suffixes += ["__" + s for s in dunder]
        self.assertSetEqual(set(suffixes), set(suffixes_expected))

    def _check_warning(self, item, message_expected, check_full=True):
        self.assertEqual(item.category, DrakeDeprecationWarning)
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
        import deprecation_example as mut

        def base_deprecation():
            warnings.warn(
                "Non-drake warning", category=DeprecationWarning, stacklevel=2)

        # At this point, no other deprecations should have been thrown, so we
        # will test with the default `once` filter.
        with warnings.catch_warnings(record=True) as w:
            # Recreate warning environment.
            warnings.simplefilter('ignore', DeprecationWarning)
            warnings.simplefilter('once', DrakeDeprecationWarning)
            # TODO(eric.cousineau): Also different behavior here...
            # Is `unittest` setting a non-standard warning filter???
            base_deprecation()  # Should not appear.
            obj = mut.ExampleClass()
            # Call each deprecated method / property repeatedly; it should only
            # warn once per unique line of source code.
            # - Method.
            for _ in range(3):
                method = mut.ExampleClass.deprecated_method
                self.assertEqual(obj.deprecated_method(), 1)
                # The next line will not show a warning.
                self.assertEqual(method(obj), 1)
            self.assertEqual(method.__doc__, mut.ExampleClass.doc_method)
            # - Property.
            for _ in range(3):
                prop = mut.ExampleClass.deprecated_prop
                self.assertEqual(obj.deprecated_prop, 2)
                # The next line will not show a warning.
                self.assertEqual(prop.__get__(obj), 2)
            self.assertEqual(prop.__doc__, mut.ExampleClass.doc_prop)
            # - Free function.
            for _ in range(3):
                x = 50
                y = mut.deprecated_func(x)
                self.assertEqual(y, 2 * x)
            # Check warnings.
            self.assertEqual(len(w), 3, "\n".join(map(str, w)))
            self._check_warning(w[0], mut.ExampleClass.message_method)
            self._check_warning(w[1], mut.ExampleClass.message_prop)
            self._check_warning(w[2], mut.message_func)

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
            method = mut.ExampleClass.deprecated_method
            self.assertEqual(len(w), 2)
            self.assertEqual(w[0].category, DeprecationWarning)
            self.assertEqual(str(w[0].message), "Non-drake warning")
            self._check_warning(w[1], mut.ExampleClass.message_method)

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
                method = mut.ExampleClass.deprecated_method
                method_extra = mut.ExampleClass.deprecated_method
                prop = mut.ExampleClass.deprecated_prop
                prop_extra = mut.ExampleClass.deprecated_prop
            # Manually set this back to `once`.
            warnings.simplefilter("ignore", DeprecationWarning)
            warnings.simplefilter("once", DrakeDeprecationWarning)

    def test_deprecated_callable(self):
        """
        Tests low-level deprecation API for callables.

        Please see `deprecation_utility_test.py` for a unittest on
        higher-level API.
        """
        import deprecation_example.cc_module as m_new
        # Spoof module name.
        var_dict = dict(__name__="fake_module")
        _forward_callables_as_deprecated(var_dict, m_new, date="2038-01-19")
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("once", DrakeDeprecationWarning)
            obj = var_dict["ExampleCppClass"]()
            self.assertIsInstance(obj, m_new.ExampleCppClass)
            message_expected = (
                "Please use ``deprecation_example.cc_module.ExampleCppClass`` "
                "instead of ``fake_module.ExampleCppClass``.")
            self.assertEqual(len(w), 1)
            self._check_warning(w[0], message_expected)
