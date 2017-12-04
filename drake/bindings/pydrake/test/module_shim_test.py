from __future__ import print_function

import unittest
import sys
from types import ModuleType


class TestModuleShim(unittest.TestCase):
    """Tests module shim functionality. """
    def tearDown(self):
        # Remove the module.
        import module_shim_example as mod
        del sys.modules[mod.__name__]
        self.assertEqual(sys.getrefcount(mod), 2)

    def testNominal(self):
        # Test reading and writing as one would do with a normal module.
        import module_shim_example as mod
        self.assertEqual(mod.value, 1)
        mod.value += 10
        self.assertEqual(mod.value, 11)
        mod.something_new = 10
        self.assertEqual(mod.something_new, 10)
        self.assertEqual(mod.__all__, ["value", "import_type", "sub_module"])
        self.assertTrue(
            str(mod).startswith("<module 'module_shim_example' from"))

    def testImportDirect(self):
        # Test an import with direct access.
        import module_shim_example as mod
        self.assertEqual(mod.import_type, "direct")
        # Check submodule.
        self.assertTrue(isinstance(mod.sub_module, str))

    def testImportFromDirect(self):
        # Test an import via `from`.
        # N.B. This is import because `from {mod} import {var}` could end up
        # resolving `{var}` as a module.
        # @ref https://docs.python.org/3/reference/simple_stmts.html#import
        from module_shim_example import import_type
        self.assertEqual(import_type, "from_direct")
        # Test submodule behavior.
        # `from_direct` should use the `getattr` overload.
        from module_shim_example import sub_module as sub
        self.assertTrue(isinstance(sub, str))
        # A nominal import should skip `getattr`, and only use the module
        # loader.
        import module_shim_example.sub_module as new_sub
        self.assertTrue(isinstance(new_sub, ModuleType))


if __name__ == '__main__':
    result = unittest.main(exit=False).result
    assert result.wasSuccessful()

    # testImportFromAll:
    # This is not a function in the unittest class because `from x import *`
    # is invalid syntax inside functions.
    from module_shim_example import *
    assert import_type == "from_all"
