from __future__ import print_function

import unittest
import sys
from types import ModuleType
import warnings


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
        self.assertEqual(mod.__all__, ["value", "import_type", "sub_module"])
        self.assertTrue(
            str(mod).startswith("<module 'deprecation_example' from"))

    def test_module_import_direct(self):
        # Test an import with direct access.
        import deprecation_example as mod
        self.assertEqual(mod.import_type, "direct")
        # Check submodule.
        self.assertTrue(isinstance(mod.sub_module, str))

    def test_module_import_from_direct(self):
        # Test an import via `from`.
        # N.B. This is import because `from {mod} import {var}` could end up
        # resolving `{var}` as a module.
        # @ref https://docs.python.org/3/reference/simple_stmts.html#import
        from deprecation_example import import_type
        self.assertEqual(import_type, "from_direct")
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
        self.assertEquals(deprecation_example.import_type, "from_all")

    def test_module_import_exec(self):
        # Test failure with `exec`; scope locals and globals to ensure we keep
        # track of the ref count.
        temp = {}
        exec "from deprecation_example import *" in temp
        self.assertEquals(temp["import_type"], "unknown")

    def _check_warning(self, w, message_expected):
        self.assertEquals(w.category, DeprecationWarning)
        self.assertEquals(w.message.message, message_expected)

    def test_member_deprecation(self):
        from deprecation_example import ExampleClass

        with warnings.catch_warnings(record=True) as w:
            obj = ExampleClass()
            # Call each deprecated method / propery repeatedly; it should only
            # warn once per unique line of source code.
            # - Method.
            for i in range(3):
                member = ExampleClass.deprecated_1
                extra = ExampleClass.deprecated_1
            self.assertEquals(member(obj), 1)
            self.assertEquals(member.__doc__, ExampleClass.doc_1)
            # - Property.
            for i in range(3):
                member = ExampleClass.deprecated_2
                extra = ExampleClass.deprecated_2
            self.assertEquals(member.__doc__, ExampleClass.doc_2)
            self.assertEquals(member.__get__(obj), 2)
            # Check warnings.
            self.assertEquals(len(w), 4)
            self._check_warning(w[0], ExampleClass.message_1)
            self._check_warning(w[1], ExampleClass.message_1)
            self._check_warning(w[2], ExampleClass.message_2)
            self._check_warning(w[3], ExampleClass.message_2)

        # Uncomment this to manually inspect the warnings.
        show_warnings = False
        if show_warnings:
            warnings.resetwarnings()
            member = ExampleClass.deprecated_1
            extra = ExampleClass.deprecated_1
            member = ExampleClass.deprecated_2
            extra = ExampleClass.deprecated_2
