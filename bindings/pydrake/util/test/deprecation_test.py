from __future__ import print_function

import unittest
import sys
from types import ModuleType


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


if __name__ == '__main__':
    result = unittest.main(exit=False).result
    assert result.wasSuccessful()

    # test_module_import_from_all:
    # This is not a function in the unittest class because `from x import *`
    # is invalid syntax inside functions.
    from deprecation_example import *
    assert import_type == "from_all"
